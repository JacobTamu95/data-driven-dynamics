"""
 *
 * Copyright (c) 2021 Manuel Yves Galliker
 *               2021 Autonomous Systems Lab ETH Zurich
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Data Driven Dynamics nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *

The model class contains properties shared between all models and shgall simplyfy automated checks and the later
export to a sitl gazebo model by providing a unified interface for all models. """

__author__ = "Manuel Yves Galliker, Julius Schlapbach"
__maintainer__ = "Manuel Yves Galliker"
__license__ = "BSD 3"

from progress.bar import Bar
import pandas as pd
import math
import time
import yaml
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import os
from src.models.model_config import ModelConfig
from src.tools.ulog_tools import load_ulog, pandas_from_topic
from src.tools.dataframe_tools import compute_flight_time, resample_dataframe_list
from src.tools.quat_utils import quaternion_to_rotation_matrix


class DataHandler(object):
    visual_dataframe_selector_config_dict = {
        "x_axis_col": "timestamp",
        "sub_plt1_data": ["q0", "q1", "q2", "q3"],
        "sub_plt2_data": ["u0", "u1", "u2", "u3"],
    }

    def __init__(self, config_file, selection_var="none"):
        print(
            "==============================================================================="
        )
        print(
            "                              Data Processing                                  "
        )
        print(
            "==============================================================================="
        )
        self.config = ModelConfig(config_file)
        config_dict = self.config.dynamics_model_config

        assert type(config_dict) is dict, "req_topics_dict input must be a dict"
        assert bool(config_dict), "req_topics_dict can not be empty"
        self.config_dict = config_dict
        self.resample_freq = config_dict["resample_freq"]
        self.estimate_angular_acceleration = config_dict[
            "estimate_angular_acceleration"
        ]
        print("Resample frequency: ", self.resample_freq, "Hz")
        self.req_topics_dict = config_dict["data"]["required_ulog_topics"]

        if selection_var != "none":
            split = selection_var.split("/")
            assert (
                len(split) == 2
            ), "Setpoint variable must be of the form: topic_name/variable_name"

            topic_name = split[0]
            variable_name = split[1]

            if topic_name in self.req_topics_dict.keys():
                assert (
                    "ulog_name" in self.req_topics_dict[topic_name].keys()
                    and "dataframe_name" in self.req_topics_dict[topic_name].keys()
                ), "Topic already exists but does not have the required keys"

                if variable_name in self.req_topics_dict[topic_name]["ulog_name"]:
                    raise AttributeError(
                        "Please only variables for setpoint data selection that are not used in a different context for system identification"
                    )

                assert (
                    "timestamp" in self.req_topics_dict[topic_name]["ulog_name"]
                    and "timestamp"
                    in self.req_topics_dict[topic_name]["dataframe_name"]
                ), "Topic already exists but does not have the required timestamp key"

                self.req_topics_dict[topic_name]["ulog_name"].append(variable_name)
                self.req_topics_dict[topic_name]["dataframe_name"].append(variable_name)

            else:
                self.req_topics_dict[topic_name] = {
                    "ulog_name": ["timestamp", variable_name],
                    "dataframe_name": ["timestamp", variable_name],
                }

            print(
                "Augmented required topics list with setpoint variable:",
                variable_name,
                "from topic",
                topic_name,
            )

        self.req_dataframe_topic_list = config_dict["data"]["req_dataframe_topic_list"]

        self.estimate_forces = config_dict["estimate_forces"]
        self.estimate_moments = config_dict["estimate_moments"]

        # used to generate a dict with the resulting coefficients later on.
        self.coef_name_list = []
        self.result_dict = {}

    def loadLogs(self, rel_data_path):
        self.rel_data_path = rel_data_path
        if os.path.isdir(rel_data_path):
            self.data_df = pd.DataFrame()
            for filename in os.listdir(rel_data_path):
                self.loadLogFile(os.path.join(rel_data_path, filename))

        else:
            if not self.loadLogFile(rel_data_path):
                raise TypeError("File extension needs to be either csv or ulg")

    def loadLogFile(self, rel_data_path):
        if rel_data_path.endswith(".csv"):
            print("Loading CSV file: ", rel_data_path)
            self.data_df = pd.read_csv(rel_data_path, index_col=0)
            print("Loading topics: ", self.req_dataframe_topic_list)
            for req_topic in self.req_dataframe_topic_list:
                assert req_topic in self.data_df, "missing topic in loaded csv: " + str(
                    req_topic
                )
            return True

        elif rel_data_path.endswith(".ulg"):
            print("Loading uLog file: ", rel_data_path)
            ulog = load_ulog(rel_data_path)
            print("Loading topics:")
            for req_topic in self.req_topics_dict:
                print(req_topic)
            self.check_ulog_for_req_topics(ulog)

            # compute flight time based on the landed topic
            landed_df = pandas_from_topic(ulog, ["vehicle_land_detected"])
            fts = compute_flight_time(landed_df)

            if len(fts) == 1:
                self.data_df = self.compute_resampled_dataframe(ulog, fts[0])
            else:
                # LOCAL FIX: the original loop called DataFrame.append, which was
                # removed in pandas 2.0. Even on pandas 1.x it was a no-op bug --
                # append returns a new frame rather than mutating in place, so every
                # segment after the first was silently discarded.
                # compute_resampled_dataframe already concatenates internally when
                # handed the full list of flight times, so just pass it through.
                self.data_df = self.compute_resampled_dataframe(ulog, fts)

            return True

        else:
            return False

    def check_ulog_for_req_topics(self, ulog):
        for topic_type in self.req_topics_dict.keys():
            try:
                topic_dict = self.req_topics_dict[topic_type]
                if "id" in topic_dict.keys():
                    id = topic_dict["id"]
                    topic_type_data = ulog.get_dataset(topic_type, id)
                else:
                    topic_type_data = ulog.get_dataset(topic_type)
            except:
                print("Missing topic type: ", topic_type)
                exit(1)
            topic_type_data = topic_type_data.data
            ulog_topic_list = self.req_topics_dict[topic_type]["ulog_name"]
            for topic_index in range(len(ulog_topic_list)):
                try:
                    topic = ulog_topic_list[topic_index]
                    topic_data = topic_type_data[topic]
                except:
                    print("Missing topic: ", topic_type, ulog_topic_list[topic_index])
                    exit(1)
        return

    def compute_resampled_dataframe(self, ulog, fts):
        print("Starting data resampling of topic types: ", self.req_topics_dict.keys())
        # setup object to crop dataframes for flight data
        df_list = []
        topic_type_bar = Bar("Resampling", max=len(self.req_topics_dict.keys()))

        # getting data
        for topic_type in self.req_topics_dict.keys():
            topic_dict = self.req_topics_dict[topic_type]

            # Only the columns named in the config participate in the NaN check --
            # see pandas_from_topic. Without this, topics that carry unused NaN
            # array slots (e.g. actuator_motors on a 4-rotor airframe) come back
            # completely empty.
            if "id" in topic_dict.keys():
                id = topic_dict["id"]
                curr_df = pandas_from_topic(
                    ulog, [topic_type], id, columns=topic_dict["ulog_name"]
                )
            else:
                curr_df = pandas_from_topic(
                    ulog, [topic_type], columns=topic_dict["ulog_name"]
                )

            curr_df = curr_df[topic_dict["ulog_name"]]
            if "dataframe_name" in topic_dict.keys():
                assert len(topic_dict["dataframe_name"]) == len(
                    topic_dict["ulog_name"]
                ), (
                    "could not rename topics of type",
                    topic_type,
                    "due to rename list not having an entry for every topic.",
                )
                curr_df.columns = topic_dict["dataframe_name"]
            topic_type_bar.next()
            if (
                topic_type == "vehicle_angular_velocity"
                and self.estimate_angular_acceleration
            ):
                ang_vel_mat = curr_df[
                    ["ang_vel_x", "ang_vel_y", "ang_vel_z"]
                ].to_numpy()
                time_in_secods_np = curr_df[["timestamp"]].to_numpy() / 1000000
                time_in_secods_np = time_in_secods_np.flatten()
                ang_acc_np = np.gradient(ang_vel_mat, time_in_secods_np, axis=0)
                topic_type_bar.next()
                curr_df[["ang_acc_b_x", "ang_acc_b_y", "ang_acc_b_z"]] = ang_acc_np

            df_list.append(curr_df)

        topic_type_bar.finish()

        # Check if actuator topics are empty
        if not fts:
            print("could not select flight time due to missing actuator topic")
            exit(1)

        # Conditioning (delay alignment, matched filtering, differentiation) is
        # applied PER SEGMENT and before the concat. Doing it after, as the code
        # used to, runs the filter and np.gradient straight across the join
        # between two flight segments that may be minutes apart in wall time,
        # fabricating an enormous acceleration at every boundary.
        if isinstance(fts, list):
            resampled_df = pd.concat(
                [
                    self.condition_segment(
                        resample_dataframe_list(df_list, ft, self.resample_freq)
                    )
                    for ft in fts
                ],
                ignore_index=True,
            )
        else:
            resampled_df = self.condition_segment(
                resample_dataframe_list(df_list, fts, self.resample_freq)
            )
        topic_type_bar.next()

        return resampled_df.dropna()

    def actuator_columns(self):
        """Dataframe columns carrying actuator channels, per the config."""
        cols = []
        for topic_dict in self.req_topics_dict.values():
            if "actuator_type" not in topic_dict:
                continue
            names = topic_dict.get("dataframe_name", topic_dict["ulog_name"])
            cols += [
                name
                for name, kind in zip(names, topic_dict["actuator_type"])
                if kind != "timestamp"
            ]
        return cols

    def condition_segment(self, df):
        """Delay-align, band-match and differentiate one contiguous segment.

        Two problems are being solved here, both of which bias the fit rather
        than merely blur it.

        1. TRANSPORT DELAY. A motor command does not become a moment
           instantaneously -- ESC dead time plus rotor spin-up puts the response
           tens of milliseconds behind the command. Regressing an unshifted
           command against the response it caused is a straightforward
           misalignment; on a real grazer log the roll correlation runs 0.19
           unshifted against 0.65 at the correct shift.

        2. BAND MISMATCH. Least squares assumes both sides of the regression
           describe the same signal. The old code low-passed only the TARGET
           (a 33-sample boxcar on angular velocity, 0.33 s at 100 Hz, first null
           at 3 Hz and sign-inverting sidelobes above it) and left the regressor
           at full bandwidth. Every regressor sample above 3 Hz was then paired
           with a target that had been zeroed or phase-flipped, and the estimator
           can only read that as "this input produces no output".

           Worse, above the motor bandwidth the rate controller's D-term makes
           the command a function of gyro noise rather than its cause, so the
           true correlation there is NEGATIVE. Including that band at all drags
           the moment coefficients toward zero regardless of filtering.

        The fix is one zero-phase low-pass, at a cutoff inside the band where
        command and response are actually coherent, applied identically to
        BOTH sides. filtfilt is used rather than a causal filter precisely
        because it adds no phase of its own -- a phase shift here would
        reintroduce the misalignment that (1) exists to remove.

        Opt in per config; without a `signal_conditioning` block the original
        boxcar path runs unchanged so existing configs reproduce exactly.
        """
        cfg = self.config_dict.get("signal_conditioning", None)

        if cfg is None:
            return self.legacy_angular_acceleration(df)

        dt = 1.0 / self.resample_freq
        nyquist = 0.5 * self.resample_freq

        # (1) Shift actuator channels FORWARD in time so each command lines up
        # with the response it produced.
        delay_s = cfg.get("actuator_delay_s", 0.0)
        shift = int(round(delay_s / dt))
        if shift > 0:
            for col in self.actuator_columns():
                if col in df:
                    df[col] = df[col].shift(shift)
            # The first `shift` rows now have no command to pair with.
            df = df.iloc[shift:].reset_index(drop=True)

        # (2) One matched zero-phase band-pass over every physical channel.
        #
        # The HIGH-PASS side matters as much as the low. The lever regressor is
        # a differential of the four rotors, so common-mode hover thrust already
        # cancels -- but what survives at DC is the static trim differential from
        # an off-centre centre of gravity. That is a large constant in the
        # regressor paired with a zero-mean target, because the model carries no
        # CG-offset term able to explain a steady moment. Least squares can only
        # shrink the coefficient to reconcile them. Removing DC costs nothing
        # (a constant carries no information about a dynamic coefficient) and
        # removes that bias.
        lo = cfg.get("highpass_cutoff_hz", None)
        hi = cfg.get("lowpass_cutoff_hz", None)
        if lo is not None or hi is not None:
            for name, f in (("highpass_cutoff_hz", lo), ("lowpass_cutoff_hz", hi)):
                assert f is None or 0 < f < nyquist, (
                    "%s (%s) must be between 0 and the Nyquist frequency (%s) "
                    "implied by resample_freq %s"
                    % (name, f, nyquist, self.resample_freq)
                )
            assert lo is None or hi is None or lo < hi, (
                "highpass_cutoff_hz (%s) must be below lowpass_cutoff_hz (%s)"
                % (lo, hi)
            )
            order = cfg.get("lowpass_order", 4)
            if lo is not None and hi is not None:
                b, a = signal.butter(order, [lo / nyquist, hi / nyquist], btype="band")
            elif hi is not None:
                b, a = signal.butter(order, hi / nyquist)
            else:
                b, a = signal.butter(order, lo / nyquist, btype="high")
            # filtfilt runs the filter forwards and backwards, so it needs a
            # comfortable margin of samples at each end.
            if len(df) <= 3 * max(len(a), len(b)):
                print(
                    "Warning: segment of %d samples is too short to filter; "
                    "leaving it unconditioned." % len(df)
                )
            else:
                skip = {"timestamp", "landed"}
                for col in df.columns:
                    if col in skip or not np.issubdtype(df[col].dtype, np.floating):
                        continue
                    df[col] = signal.filtfilt(b, a, df[col].to_numpy())

        # (3) Differentiate the already-filtered rates. No extra smoothing --
        # the matched filter above is the only band limit, which is what keeps
        # the target in the same band as the regressor.
        if self.estimate_angular_acceleration:
            t = df["timestamp"].to_numpy() / 1000000
            ang_vel = df[["ang_vel_x", "ang_vel_y", "ang_vel_z"]].to_numpy()
            df[["ang_acc_b_x", "ang_acc_b_y", "ang_acc_b_z"]] = np.gradient(
                ang_vel, t, axis=0
            )

        return df

    def legacy_angular_acceleration(self, df):
        """Original 33-sample boxcar path, kept so existing configs reproduce.

        Retained for backward compatibility only. See condition_segment for why
        this biases the moment coefficients; prefer a `signal_conditioning`
        block in new configs.
        """
        if not self.estimate_angular_acceleration:
            return df

        ang_vel_mat = df[["ang_vel_x", "ang_vel_y", "ang_vel_z"]].to_numpy()
        for i in range(3):
            ang_vel_mat[:, i] = (
                np.convolve(ang_vel_mat[:, i], np.ones(33), mode="same") / 33
            )

        time_in_secods_np = df[["timestamp"]].to_numpy() / 1000000
        time_in_secods_np = time_in_secods_np.flatten()
        ang_acc_np = np.gradient(ang_vel_mat, time_in_secods_np, axis=0)
        df[["ang_acc_b_x", "ang_acc_b_y", "ang_acc_b_z"]] = ang_acc_np
        return df

    def visually_select_data(self, plot_config_dict=None):
        print(
            "==============================================================================="
        )
        print(
            "                           Data Selection Enabled                              "
        )
        print(
            "==============================================================================="
        )
        from visual_dataframe_selector.data_selector import select_visual_data

        print("Number of data samples before cropping: ", self.data_df.shape[0])
        self.data_df = select_visual_data(
            self.data_df, self.visual_dataframe_selector_config_dict
        )

    def get_dataframes(self):
        return self.data_df

    def visualize_data(self):
        def plot_scatter(
            ax, title, dataframe_x, dataframe_y, dataframe_z, color="blue"
        ):
            ax.scatter(
                self.data_df[dataframe_x],
                self.data_df[dataframe_y],
                self.data_df[dataframe_z],
                s=10,
                facecolor=color,
                lw=0,
                alpha=0.1,
            )
            ax.set_title(title)
            ax.set_xlabel(dataframe_x)
            ax.set_ylabel(dataframe_y)
            ax.set_zlabel(dataframe_z)

        num_plots = 2
        fig = plt.figure("Data Visualization")
        ax1 = fig.add_subplot(num_plots, 2, 1, projection="3d")
        plot_scatter(ax1, "Local Velocity", "vx", "vy", "vz")

        ax2 = fig.add_subplot(num_plots, 2, 2, projection="3d")
        plot_scatter(ax2, "Body Acceleration", "acc_b_x", "acc_b_y", "acc_b_z", "red")

        ax3 = fig.add_subplot(num_plots, 2, 3, projection="3d")
        plot_scatter(
            ax3, "Body Angular Velocity", "ang_vel_x", "ang_vel_y", "ang_vel_z", "red"
        )

        ax4 = fig.add_subplot(num_plots, 2, 4, projection="3d")
        plot_scatter(
            ax4,
            "Body Angular Acceleration",
            "ang_acc_b_x",
            "ang_acc_b_y",
            "ang_acc_b_z",
            "red",
        )
        plt.show(block=False)
