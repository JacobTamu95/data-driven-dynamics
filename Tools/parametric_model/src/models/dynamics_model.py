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
"""

__author__ = "Manuel Yves Galliker"
__maintainer__ = "Manuel Yves Galliker"
__license__ = "BSD 3"

from src.tools.math_tools import cropped_sym_sigmoid
from src.tools.quat_utils import quaternion_to_rotation_matrix
from src.tools.dataframe_tools import resample_dataframe_list
from src.tools.ulog_tools import load_ulog, pandas_from_topic
from .model_plots import model_plots, aerodynamics_plots, linear_model_plots
from .rotor_models import (
    RotorModel,
    LinearRotorModel,
    BiDirectionalRotorModel,
    TiltingRotorModel,
    ChangingAxisRotorModel,
)
import matplotlib.pyplot as plt
from scipy.linalg import block_diag
import src.optimizers as optimizers
import numpy as np
from scipy import signal
import yaml
import time
import warnings
import math
import pandas as pd
from progress.bar import Bar

""" The model class contains properties shared between all models and shgall simplyfy automated checks and the later
export to a sitl gazebo model by providing a unified interface for all models. """


class DynamicsModel:
    def __init__(self, config_dict, normalization=True):
        assert type(config_dict) is dict, "req_topics_dict input must be a dict"
        assert bool(config_dict), "req_topics_dict can not be empty"
        self.model_name = "unknown_model"
        self.config_dict = config_dict
        self.resample_freq = config_dict["resample_freq"]
        self.optimizer_config = config_dict["optimizer_config"]
        self.req_topics_dict = config_dict["data"]["required_ulog_topics"]
        self.req_dataframe_topic_list = config_dict["data"]["req_dataframe_topic_list"]

        self.visual_dataframe_selector_config_dict = {
            "x_axis_col": "timestamp",
            "sub_plt1_data": ["q0", "q1", "q2", "q3"],
            "sub_plt2_data": ["u0", "u1", "u2", "u3"],
        }

        self.estimate_forces = config_dict["estimate_forces"]
        self.estimate_moments = config_dict["estimate_moments"]
        self.apply_normalization = normalization

        # used to generate a dict with the resulting coefficients later on.
        self.coef_name_list = []
        self.y_dict = {}
        self.coef_dict = {}
        self.result_dict = {}

    def prepare_regression_matrices(self):
        if "V_air_body_x" not in self.data_df:
            if self.apply_normalization:
                self.normalize_actuators()
            self.compute_airspeed_from_groundspeed(["vx", "vy", "vz"])

        # Rotor features
        angular_vel_mat = self.data_df[
            ["ang_vel_x", "ang_vel_y", "ang_vel_z"]
        ].to_numpy()
        self.compute_rotor_features(self.rotor_config_dict, angular_vel_mat)

        if self.estimate_forces and self.estimate_moments:
            self.prepare_force_regression_matrices()
            self.prepare_moment_regression_matrices()

        elif self.estimate_forces:
            self.prepare_force_regression_matrices()

        elif self.estimate_moments:
            self.prepare_moment_regression_matrices()

        else:
            raise ValueError("Neither Forces nor Moments estimation activated")

        return

    def prepare_force_regression_matrices(self):
        raise NotImplementedError()

    def prepare_moment_regression_matrices(self):
        raise NotImplementedError()

    def assemble_regression_matrices(self, measurements):
        sizes = [len(self.y_dict[i].keys()) for i in measurements]
        y = np.empty(sum(sizes) * self.n_samples)
        i = 0
        for m in measurements:
            for k in self.y_dict[m].keys():
                y[i * self.n_samples : (i + 1) * self.n_samples] = self.data_df[
                    self.y_dict[m][k]
                ]
                i += 1

        coef_list = []

        for i in self.coef_dict.keys():
            for m in measurements:
                if m in self.coef_dict[i]:
                    coef_list.append(i)

        X = np.zeros((len(measurements) * self.n_samples * 3, len(coef_list)))
        for coef_index, coef in enumerate(coef_list):
            for i_index, i in enumerate(measurements):
                for j_index, j in enumerate(["x", "y", "z"]):
                    try:
                        pos = self.n_samples * (i_index * 3 + j_index)
                        key = self.coef_dict[coef][i][j]
                        X[pos : pos + self.n_samples, coef_index] = self.data_df[key]
                    except:
                        KeyError

        X, y = self.prefilter_regression(X, y)
        X, coef_list = self.add_moment_trim_offsets(X, coef_list, measurements)

        return X, y, coef_list

    def add_moment_trim_offsets(self, X, coef_list, measurements):
        """Give the moment fit one constant per body axis to absorb steady trim.

        A multirotor in equilibrium carries a small permanent differential
        between its rotors, because the centre of gravity never sits exactly on
        the geometric centre. That shows up as a large constant in the lever
        regressor paired with a zero-mean target -- angular acceleration
        averages to zero over a flight, while the trim differential does not.

        The model has no term able to explain a steady moment, so least squares
        can only reconcile the two by shrinking the dynamic coefficient. Three
        free constants let it explain the offset honestly instead, which leaves
        the lever coefficient free to describe what it should: the response to
        CHANGES in differential thrust.

        This is the reason to prefer an intercept over simply high-passing the
        regression. High-passing also removes the offset, but it throws away
        every genuine low-frequency correlation along with it. The intercept
        removes only the constant.

        Appended AFTER prefiltering: a constant column is exactly what a
        high-pass annihilates, and filtering a constant is meaningless anyway.
        """
        if not self.config_dict.get("moment_trim_offsets", False):
            return X, coef_list
        if "rot" not in measurements:
            return X, coef_list

        n = self.n_samples
        rot_index = list(measurements).index("rot")
        offsets = np.zeros((X.shape[0], 3))
        for axis_index in range(3):
            start = n * (rot_index * 3 + axis_index)
            offsets[start : start + n, axis_index] = 1.0

        names = ["c_m_trim_x", "c_m_trim_y", "c_m_trim_z"]
        print("Adding moment trim offsets: %s" % ", ".join(names))
        return np.hstack((X, offsets)), coef_list + names

    def prefilter_regression(self, X, y):
        """Band-limit both sides of the regression to a common, coherent band.

        This is the classical system-identification prefilter. Because it is a
        linear operator applied IDENTICALLY to every column of X and to y, it
        leaves the underlying relationship untouched -- it only reweights which
        frequencies the least-squares fit is allowed to listen to.

        It has to happen HERE, on assembled features, not on the raw dataframe
        columns. The rotor features are nonlinear in the actuator input (thrust
        goes as u^2), and a band-pass makes u zero-mean, so squaring a filtered
        u produces something unrelated to a filtered u^2. Filter the feature.

        Why band-limit at all: the fit is only valid where command and response
        are coherent. Below that band a static CG-trim differential is a large
        constant in the lever regressor with no term in the model able to
        explain the steady moment it produces. Above it, past the motor
        bandwidth, the rate controller's D-term makes the command a function of
        gyro noise rather than its cause, so the true correlation there is
        NEGATIVE. Full-band least squares averages the honest middle against
        both, and lands near zero.

        Blocks are filtered independently: X and y are laid out as contiguous
        n_samples-long time series, one per (measurement, axis) pair.
        """
        cfg = self.config_dict.get("regression_prefilter", None)
        if cfg is None:
            return X, y

        fs = self.config_dict["resample_freq"]
        nyquist = 0.5 * fs
        lo = cfg.get("highpass_cutoff_hz", None)
        hi = cfg.get("lowpass_cutoff_hz", None)
        order = cfg.get("order", 4)

        for name, f in (("highpass_cutoff_hz", lo), ("lowpass_cutoff_hz", hi)):
            assert f is None or 0 < f < nyquist, (
                "%s (%s) must be between 0 and the Nyquist frequency (%s) "
                "implied by resample_freq %s" % (name, f, nyquist, fs)
            )
        assert lo is None or hi is None or lo < hi, (
            "highpass_cutoff_hz (%s) must be below lowpass_cutoff_hz (%s)"
            % (lo, hi)
        )
        if lo is None and hi is None:
            return X, y

        # Second-order sections, not transfer-function coefficients: a narrow
        # band-pass at a low normalized frequency is badly conditioned in (b, a)
        # form at anything past order 2.
        if lo is not None and hi is not None:
            sos = signal.butter(
                order, [lo / nyquist, hi / nyquist], btype="band", output="sos"
            )
        elif hi is not None:
            sos = signal.butter(order, hi / nyquist, output="sos")
        else:
            sos = signal.butter(order, lo / nyquist, btype="high", output="sos")

        n = self.n_samples
        if n <= 12 * order:
            print(
                "Warning: %d samples per block is too short to prefilter; "
                "skipping." % n
            )
            return X, y

        print(
            "Prefiltering regression: %s-%s Hz, order %d, %d blocks of %d samples"
            % (lo, hi, order, X.shape[0] // n, n)
        )
        # sosfiltfilt is zero-phase, so it introduces no lag of its own -- which
        # matters because an actuator delay has already been aligned out by hand.
        for start in range(0, X.shape[0], n):
            X[start : start + n, :] = signal.sosfiltfilt(
                sos, X[start : start + n, :], axis=0
            )
        for start in range(0, y.shape[0], n):
            y[start : start + n] = signal.sosfiltfilt(sos, y[start : start + n])

        return X, y

    def get_topic_list_from_topic_type(self, topic_type):
        topic_type_name_dict = self.req_topics_dict[topic_type]
        if "dataframe_name" in topic_type_name_dict.keys():
            topic_columns = topic_type_name_dict["dataframe_name"].copy()
        else:
            topic_columns = topic_type_name_dict["ulog_name"].copy()
        topic_columns.remove("timestamp")
        return topic_columns

    def compute_airspeed_from_groundspeed(self, airspeed_topic_list):
        groundspeed_ned_mat = (self.data_df[airspeed_topic_list]).to_numpy()
        airspeed_body_mat = self.rot_to_body_frame(groundspeed_ned_mat)
        aoa_vec = np.zeros((airspeed_body_mat.shape[0], 1))
        sideslip_vec = np.zeros((airspeed_body_mat.shape[0], 1))
        for i in range(airspeed_body_mat.shape[0]):
            aoa_vec[i, :] = math.atan2(airspeed_body_mat[i, 2], airspeed_body_mat[i, 0])
            sideslip_vec[i, :] = math.atan2(
                airspeed_body_mat[i, 1], airspeed_body_mat[i, 0]
            )

        airspeed_body_mat = np.hstack((airspeed_body_mat, aoa_vec, sideslip_vec))
        airspeed_body_df = pd.DataFrame(
            airspeed_body_mat,
            columns=[
                "V_air_body_x",
                "V_air_body_y",
                "V_air_body_z",
                "angle_of_attack",
                "angle_of_sideslip",
            ],
        )
        self.data_df = pd.concat([self.data_df, airspeed_body_df], axis=1, join="inner")

    def compute_body_rotation_features(self, angular_vel_topic_list):
        """Include the moment contribution due to rotation body frame:
        w x Iw = X_body_rot * v
        Where v = (I_y-I_z, I_z-I_x, I_x- I_y)^T
        is comprised of the inertia moments we want to estimate
        """
        angular_vel_mat = (self.data_df[angular_vel_topic_list]).to_numpy()
        X_body_rot = np.zeros((3 * angular_vel_mat.shape[0], 3))
        X_body_rot_coef_list = ["I_yy-I_zz", "I_zz-I_xx", "I_xx- I_yy"]
        for i in range(angular_vel_mat.shape[0]):
            X_body_rot[3 * i, 0] = angular_vel_mat[i, 1] * angular_vel_mat[i, 2]
            X_body_rot[3 * i + 1, 0] = angular_vel_mat[i, 2] * angular_vel_mat[i, 0]
            X_body_rot[3 * i + 2, 0] = angular_vel_mat[i, 0] * angular_vel_mat[i, 1]
        return X_body_rot, X_body_rot_coef_list

    def normalize_actuators(
        self, actuator_topic_types=None, control_outputs_used=False
    ):
        # u : normalize actuator output from pwm to be scaled between 0 and 1
        # To be adjusted using parameters:

        if actuator_topic_types is None:
            # Derive the actuator topics from the config rather than assuming
            # "actuator_outputs". Any topic declaring actuator_type carries
            # actuator channels, so this also supports actuator_motors.
            actuator_topic_types = [
                topic
                for topic, spec in self.req_topics_dict.items()
                if "actuator_type" in spec
            ] or ["actuator_outputs"]

        # Range of the logged actuator signal, used to map it onto the 0..1 that
        # the rotor models expect. actuator_outputs is PWM in microseconds, but
        # actuator_motors is already normalized -- for that case set
        # min_output/max_output to 0/1 in the config to make this a pass-through.
        # This should probably be adapted in the future to allow different values for each actuator specified in the config.
        norm_config = self.config_dict.get("actuator_normalization", None)
        if norm_config is not None:
            self.min_output = norm_config["min_output"]
            self.max_output = norm_config["max_output"]
            self.trim_output = norm_config["trim_output"]
        elif control_outputs_used:
            self.min_output = -1
            self.max_output = 1.01
            self.trim_output = 0
        else:
            self.min_output = 0
            self.max_output = 2000
            self.trim_output = 1500

        self.actuator_columns = []
        self.actuator_type = []

        for topic_type in actuator_topic_types:
            self.actuator_columns += self.get_topic_list_from_topic_type(topic_type)
            self.actuator_type += self.req_topics_dict[topic_type]["actuator_type"]
            self.actuator_type.remove("timestamp")

        for i in range(len(self.actuator_columns)):
            actuator_data = self.data_df[self.actuator_columns[i]].to_numpy()
            if self.actuator_type[i] == "motor":
                for j in range(actuator_data.shape[0]):
                    if actuator_data[j] < self.min_output:
                        actuator_data[j] = 0
                    else:
                        actuator_data[j] = (actuator_data[j] - self.min_output) / (
                            self.max_output - self.min_output
                        )
            elif (
                self.actuator_type[i] == "control_surface"
                or self.actuator_type[i] == "bi_directional_motor"
            ):
                for j in range(actuator_data.shape[0]):
                    if actuator_data[j] < self.min_output:
                        actuator_data[j] = 0
                    else:
                        actuator_data[j] = (
                            2
                            * (actuator_data[j] - self.trim_output)
                            / (self.max_output - self.min_output)
                        )
            else:
                print("actuator type unknown:", self.actuator_type[i])
                print("normalization failed")
                exit(1)
            self.data_df[self.actuator_columns[i]] = actuator_data

    def initialize_rotor_model(self, rotor_config_dict, angular_vel_mat=None):
        valid_rotor_types = [
            "RotorModel",
            "ChangingAxisRotorModel",
            "BiDirectionalRotorModel",
            "TiltingRotorModel",
            "LinearRotorModel",
        ]
        rotor_input_name = rotor_config_dict["dataframe_name"]
        u_vec = self.data_df[rotor_input_name].to_numpy()
        if "rotor_type" not in rotor_config_dict.keys():
            # Set default rotor model
            rotor_type = "RotorModel"
            print("no Rotor model specified for ", rotor_input_name)
            print("Selecting default: RotorModel")
        else:
            rotor_type = rotor_config_dict["rotor_type"]

        if rotor_type == "RotorModel":
            rotor = RotorModel(
                rotor_config_dict,
                u_vec,
                self.v_airspeed_mat,
                angular_vel_mat=angular_vel_mat,
            )
        elif rotor_type == "LinearRotorModel":
            rotor = LinearRotorModel(u_vec)
        elif rotor_type == "ChangingAxisRotorModel":
            rotor = ChangingAxisRotorModel(
                rotor_config_dict,
                u_vec,
                self.v_airspeed_mat,
                angular_vel_mat=angular_vel_mat,
            )
        elif rotor_type == "BiDirectionalRotorModel":
            rotor = BiDirectionalRotorModel(
                rotor_config_dict,
                u_vec,
                self.v_airspeed_mat,
                angular_vel_mat=angular_vel_mat,
            )
        elif rotor_type == "TiltingRotorModel":
            tilt_actuator_df_name = rotor_config_dict["tilt_actuator_dataframe_name"]
            tilt_actuator_vec = self.data_df[tilt_actuator_df_name]
            rotor = TiltingRotorModel(
                rotor_config_dict,
                u_vec,
                self.v_airspeed_mat,
                tilt_actuator_vec,
                angular_vel_mat=angular_vel_mat,
            )
        else:
            print(rotor_type, " is not a valid rotor model.")
            print("Valid rotor models are: ", valid_rotor_types)
            print("Adapt your config file to a valid rotor model!")
            exit(1)

        return rotor

    def compute_rotor_features(self, rotors_config_dict, angular_vel_mat=None):
        self.v_airspeed_mat = self.data_df[
            ["V_air_body_x", "V_air_body_y", "V_air_body_z"]
        ].to_numpy()
        self.rotor_dict = {}

        for rotor_group in rotors_config_dict.keys():
            rotor_group_list = rotors_config_dict[rotor_group]
            self.rotor_dict[rotor_group] = {}
            if self.estimate_forces:
                # ! issue here
                if (
                    rotors_config_dict[rotor_group][0]["rotor_type"]
                    == "LinearRotorModel"
                ):
                    X_force_collector = np.zeros((self.n_samples, 3))
                else:
                    X_force_collector = np.zeros((self.n_samples, 3 * 3))
            if self.estimate_moments:
                if (
                    rotors_config_dict[rotor_group][0]["rotor_type"]
                    == "LinearRotorModel"
                ):
                    X_moment_collector = np.zeros((self.n_samples, 3))
                else:
                    X_moment_collector = np.zeros((self.n_samples, 3 * 5))
            for rotor_config_dict in rotor_group_list:
                rotor = self.initialize_rotor_model(rotor_config_dict, angular_vel_mat)
                self.rotor_dict[rotor_group][
                    rotor_config_dict["dataframe_name"]
                ] = rotor

                if self.estimate_forces:
                    (
                        X_force_curr,
                        coef_dict_force,
                        col_names_force,
                    ) = rotor.compute_actuator_force_matrix()
                    X_force_collector = X_force_collector + X_force_curr
                    # Include rotor group name in coefficient names:
                    for i in range(len(col_names_force)):
                        col_names_force[i] = rotor_group + col_names_force[i]

                    for key in list(coef_dict_force.keys()):
                        coef_dict_force[rotor_group + key] = coef_dict_force.pop(key)
                        for i in ["x", "y", "z"]:
                            coef_dict_force[rotor_group + key]["lin"][i] = (
                                rotor_group
                                + coef_dict_force[rotor_group + key]["lin"][i]
                            )

                if self.estimate_moments:
                    (
                        X_moment_curr,
                        coef_dict_moment,
                        col_names_moment,
                    ) = rotor.compute_actuator_moment_matrix()
                    X_moment_collector = X_moment_collector + X_moment_curr
                    # Include rotor group name in coefficient names:
                    for i in range(len(col_names_moment)):
                        col_names_moment[i] = rotor_group + col_names_moment[i]

                    for key in list(coef_dict_moment.keys()):
                        coef_dict_moment[rotor_group + key] = coef_dict_moment.pop(key)
                        for i in ["x", "y", "z"]:
                            coef_dict_moment[rotor_group + key]["rot"][i] = (
                                rotor_group
                                + coef_dict_moment[rotor_group + key]["rot"][i]
                            )

            if self.estimate_forces:
                self.data_df[col_names_force] = X_force_collector
                self.coef_dict.update(coef_dict_force)

            if self.estimate_moments:
                self.data_df[col_names_moment] = X_moment_collector
                self.coef_dict.update(coef_dict_moment)

        return

    def rot_to_body_frame(self, vec_mat):
        """
        Rotates horizontally stacked 3D vectors from NED world frame to FRD body frame

        inputs:
        vec_mat: numpy array of dimensions (n,3),
        containing the horizontally stacked 3D vectors [x,y,z] in world frame.
        """
        vec_mat_transformed = np.zeros(vec_mat.shape)
        for i in range(vec_mat.shape[0]):
            R_world_to_body = np.linalg.inv(
                quaternion_to_rotation_matrix(self.q_mat[i, :])
            )
            vec_mat_transformed[i, :] = np.transpose(
                R_world_to_body @ np.transpose(vec_mat[i, :])
            )
        return vec_mat_transformed

    def rot_to_world_frame(self, vec_mat):
        """
        Rotates horizontally stacked 3D vectors from FRD body frame to NED world frame

        inputs:
        vec_mat: numpy array of dimensions (n,3),
        containing the horizontally stacked 3D vectors [x,y,z] in body frame.
        """
        vec_mat_transformed = np.zeros(vec_mat.shape)
        for i in range(vec_mat.shape[0]):
            R_body_to_world = quaternion_to_rotation_matrix(self.q_mat[i, :])
            vec_mat_transformed[i, :] = R_body_to_world @ vec_mat[i, :]
        return vec_mat_transformed

    def generate_model_dict(self, coefficient_list, metrics_dict, model_dict):
        assert len(self.coef_name_list) == len(coefficient_list), (
            "Length of coefficient list and coefficient name list does not match: Length of coefficient list:",
            len(coefficient_list),
            "length of coefficient name list: ",
            len(self.coef_name_list),
        )
        coefficient_list = [float(coef) for coef in coefficient_list]
        coef_dict = dict(zip(self.coef_name_list, coefficient_list))
        self.result_dict = {
            "model": model_dict,
            "coefficients": coef_dict,
            "metrics": metrics_dict,
            "number of samples": self.n_samples,
        }

    def save_result_dict_to_yaml(
        self,
        file_name="model_parameters",
        result_path="model_results/",
        results_only=False,
    ):
        timestr = time.strftime("%Y-%m-%d-%H-%M-%S")
        file_path = result_path + file_name + "_" + timestr + ".yaml"

        with open(file_path, "w") as outfile:
            yaml.dump(self.result_dict, outfile, default_flow_style=False)
            if not results_only:
                yaml.dump(self.fisher_metric, outfile, default_flow_style=False)
        print(
            "-------------------------------------------------------------------------------"
        )
        print("Complete results saved to: ")
        print(file_path)
        print(
            "-------------------------------------------------------------------------------"
        )

    def load_dataframes(self, data_frame):
        self.data_df = data_frame
        self.n_samples = self.data_df.shape[0]
        self.quaternion_df = self.data_df[["q0", "q1", "q2", "q3"]]
        self.q_mat = self.quaternion_df.to_numpy()
        print(
            "-------------------------------------------------------------------------------"
        )
        print("Initialized dataframe with the following columns: ")
        print(list(self.data_df.columns))
        print("Data contains ", self.n_samples, "timestamps.")

    def predict_model(self, opt_coefs_dict):
        print(
            "==============================================================================="
        )
        print(
            "                        Preparing Model Features                               "
        )
        print(
            "==============================================================================="
        )
        self.prepare_regression_matrices()

        configuration = []
        if self.estimate_forces:
            configuration.append("lin")
        if self.estimate_moments:
            configuration.append("rot")
        self.X, self.y, self.coef_name_list = self.assemble_regression_matrices(
            configuration
        )

        c_opt_list = []
        for coef in self.coef_name_list:
            c_opt_list.append(opt_coefs_dict[coef])

        self.initialize_optimizer()
        self.optimizer.set_optimal_coefficients(c_opt_list, self.X, self.y)
        self.generate_prediction_results()

    def estimate_model(self):
        print(
            "==============================================================================="
        )
        print(
            "                        Preparing Model Features                               "
        )
        print(
            "==============================================================================="
        )
        configuration = []
        if self.estimate_forces:
            configuration.append("lin")
        if self.estimate_moments:
            configuration.append("rot")
        self.X, self.y, self.coef_name_list = self.assemble_regression_matrices(
            configuration
        )
        self.initialize_optimizer()
        self.optimizer.estimate_parameters(self.X, self.y)
        self.generate_optimization_results()

        return

    def get_model_coeffs(self):
        metrics_dict = self.optimizer.compute_optimization_metrics()
        coef_list = self.optimizer.get_optimization_parameters()
        model_dict = {}
        assert len(self.coef_name_list) == len(coef_list), (
            "Length of coefficient list and coefficient name list does not match: Length of coefficient list:",
            len(coef_list),
            "length of coefficient name list: ",
            len(self.coef_name_list),
        )
        coefficient_list = [float(coef) for coef in coef_list]
        coef_dict = dict(zip(self.coef_name_list, coefficient_list))

        return coef_dict

    def initialize_optimizer(self):
        print(
            "==============================================================================="
        )
        print(
            "                            Initialize Optimizer                               "
        )
        print(
            "                                "
            + self.optimizer_config["optimizer_class"]
        )
        print(
            "==============================================================================="
        )

        try:
            # This will call the optimizer constructor directly from the optimizer_class
            self.optimizer = getattr(
                optimizers, self.optimizer_config["optimizer_class"]
            )(self.optimizer_config, self.coef_name_list)
        except AttributeError:
            error_str = (
                "Optimizer Class '{0}' not found, is it added to optimizers "
                "directory and optimizers/__init__.py?"
            )
            raise AttributeError(error_str)

    def generate_prediction_results(self):
        print(
            "==============================================================================="
        )
        print(
            "                            Prediction Results                                 "
        )
        print(
            "==============================================================================="
        )
        metrics_dict = self.optimizer.compute_optimization_metrics()
        coef_list = self.optimizer.get_optimization_parameters()
        model_dict = {}
        model_dict.update(self.rotor_config_dict)
        if hasattr(self, "aerodynamics_dict"):
            model_dict.update(self.aerodynamics_dict)
        self.generate_model_dict(coef_list, metrics_dict, model_dict)
        print(
            "                           Optimal Coefficients                              "
        )
        print(
            "-------------------------------------------------------------------------------"
        )
        print(yaml.dump(self.result_dict["coefficients"], default_flow_style=False))
        print(
            "-------------------------------------------------------------------------------"
        )
        print(
            "                             Prediction Metrics                                "
        )
        print(
            "-------------------------------------------------------------------------------"
        )
        print(yaml.dump(self.result_dict["metrics"], default_flow_style=False))
        self.save_result_dict_to_yaml(file_name=self.model_name, results_only=True)

    def generate_optimization_results(self):
        print(
            "==============================================================================="
        )
        print(
            "                           Optimization Results                                "
        )
        print(
            "==============================================================================="
        )
        metrics_dict = self.optimizer.compute_optimization_metrics()
        coef_list = self.optimizer.get_optimization_parameters()
        model_dict = {}
        if hasattr(self, "rotor_config_dict"):
            model_dict.update(self.rotor_config_dict)
        if hasattr(self, "aerodynamics_dict"):
            model_dict.update(self.aerodynamics_dict)
        self.generate_model_dict(coef_list, metrics_dict, model_dict)
        print(
            "                           Optimal Coefficients                              "
        )
        print(
            "-------------------------------------------------------------------------------"
        )
        print(yaml.dump(self.result_dict["coefficients"], default_flow_style=False))
        print(
            "-------------------------------------------------------------------------------"
        )
        print(
            "                            Optimization Metrics                               "
        )
        print(
            "-------------------------------------------------------------------------------"
        )
        print(yaml.dump(self.result_dict["metrics"], default_flow_style=False))
        self.save_result_dict_to_yaml(file_name=self.model_name)

    def compute_residuals(self):
        y_pred = self.optimizer.predict(self.X)
        if self.estimate_forces:
            _, y_forces, _ = self.assemble_regression_matrices(["lin"])
            y_forces_measured = np.zeros(y_forces.shape)
            y_forces_measured[0::3] = y_forces[0 : int(y_forces.shape[0] / 3)]
            y_forces_measured[1::3] = y_forces[
                int(y_forces.shape[0] / 3) : int(2 * y_forces.shape[0] / 3)
            ]
            y_forces_measured[2::3] = y_forces[
                int(2 * y_forces.shape[0] / 3) : y_forces.shape[0]
            ]

            y_forces_pred = np.zeros(y_forces.shape)
            y_forces_pred[0::3] = y_pred[0 : int(y_forces.shape[0] / 3)]
            y_forces_pred[1::3] = y_pred[
                int(y_forces.shape[0] / 3) : int(2 * y_forces.shape[0] / 3)
            ]
            y_forces_pred[2::3] = y_pred[
                int(2 * y_forces.shape[0] / 3) : y_forces.shape[0]
            ]

            error_y_forces = y_forces_pred - y_forces_measured

            stacked_error_y_forces = np.array(error_y_forces)
            acc_mat = stacked_error_y_forces.reshape((-1, 3))
            residual_force_df = pd.DataFrame(
                acc_mat,
                columns=["residual_force_x", "residual_force_y", "residual_force_z"],
            )
            self.data_df = pd.concat(
                [self.data_df, residual_force_df], axis=1, join="inner"
            ).reindex(self.data_df.index)

        if self.estimate_moments:
            _, y_moments, _ = self.assemble_regression_matrices(["rot"])

            y_moments_measured = np.zeros(y_moments.shape)
            y_moments_measured[0::3] = y_moments[0 : int(y_moments.shape[0] / 3)]
            y_moments_measured[1::3] = y_moments[
                int(y_moments.shape[0] / 3) : int(2 * y_moments.shape[0] / 3)
            ]
            y_moments_measured[2::3] = y_moments[
                int(2 * y_moments.shape[0] / 3) : y_moments.shape[0]
            ]

            y_moments_pred = np.zeros(y_moments.shape)
            y_moments_pred[0::3] = y_pred[
                y_moments.shape[0] : int(4 * y_moments.shape[0] / 3)
            ]
            y_moments_pred[1::3] = y_pred[
                int(4 * y_moments.shape[0] / 3) : int(5 * y_moments.shape[0] / 3)
            ]
            y_moments_pred[2::3] = y_pred[int(5 * y_moments.shape[0] / 3) :]

            error_y_moments = y_moments_pred - y_moments_measured

            stacked_error_y_moments = np.array(error_y_moments)
            mom_mat = stacked_error_y_moments.reshape((-1, 3))
            residual_moment_df = pd.DataFrame(
                mom_mat,
                columns=["residual_moment_x", "residual_moment_y", "residual_moment_z"],
            )
            self.data_df = pd.concat(
                [self.data_df, residual_moment_df], axis=1, join="inner"
            ).reindex(self.data_df.index)

    def plot_model_predicitons(self):
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

        y_pred = self.optimizer.predict(self.X)

        fig = plt.figure("Residual Visualization")

        model_plots.plot_airspeed_and_AoA(
            self.data_df[
                ["V_air_body_x", "V_air_body_y", "V_air_body_z", "angle_of_attack"]
            ],
            self.data_df["timestamp"],
        )

        if self.estimate_forces:
            _, y_forces, _ = self.assemble_regression_matrices(["lin"])

            y_forces_measured = np.zeros(y_forces.shape)
            y_forces_measured[0::3] = y_forces[0 : int(y_forces.shape[0] / 3)]
            y_forces_measured[1::3] = y_forces[
                int(y_forces.shape[0] / 3) : int(2 * y_forces.shape[0] / 3)
            ]
            y_forces_measured[2::3] = y_forces[
                int(2 * y_forces.shape[0] / 3) : y_forces.shape[0]
            ]

            y_forces_pred = np.zeros(y_forces.shape)
            y_forces_pred[0::3] = y_pred[0 : int(y_forces.shape[0] / 3)]
            y_forces_pred[1::3] = y_pred[
                int(y_forces.shape[0] / 3) : int(2 * y_forces.shape[0] / 3)
            ]
            y_forces_pred[2::3] = y_pred[
                int(2 * y_forces.shape[0] / 3) : y_forces.shape[0]
            ]

            model_plots.plot_force_predictions(
                y_forces_measured, y_forces_pred, self.data_df["timestamp"]
            )

            ax1 = fig.add_subplot(2, 2, 1, projection="3d")
            plot_scatter(
                ax1,
                "Residual forces [N]",
                "residual_force_x",
                "residual_force_y",
                "residual_force_z",
                "blue",
            )
            ax3 = fig.add_subplot(2, 2, 3, projection="3d")
            plot_scatter(
                ax3,
                "Measured Forces [N]",
                "measured_force_x",
                "measured_force_y",
                "measured_force_z",
                "blue",
            )

        if self.estimate_moments:
            _, y_moments, _ = self.assemble_regression_matrices(["rot"])

            y_moments_measured = np.zeros(y_moments.shape)
            y_moments_measured[0::3] = y_moments[0 : int(y_moments.shape[0] / 3)]
            y_moments_measured[1::3] = y_moments[
                int(y_moments.shape[0] / 3) : int(2 * y_moments.shape[0] / 3)
            ]
            y_moments_measured[2::3] = y_moments[
                int(2 * y_moments.shape[0] / 3) : y_moments.shape[0]
            ]

            y_moments_pred = np.zeros(y_moments.shape)
            y_moments_pred[0::3] = y_pred[
                y_moments.shape[0] : int(4 * y_moments.shape[0] / 3)
            ]
            y_moments_pred[1::3] = y_pred[
                int(4 * y_moments.shape[0] / 3) : int(5 * y_moments.shape[0] / 3)
            ]
            y_moments_pred[2::3] = y_pred[int(5 * y_moments.shape[0] / 3) :]

            model_plots.plot_moment_predictions(
                y_moments_measured, y_moments_pred, self.data_df["timestamp"]
            )

            ax2 = fig.add_subplot(2, 2, 2, projection="3d")

            plot_scatter(
                ax2,
                "Residual Moments [Nm]",
                "residual_moment_x",
                "residual_moment_y",
                "residual_moment_z",
                "blue",
            )

            ax4 = fig.add_subplot(2, 2, 4, projection="3d")

            plot_scatter(
                ax4,
                "Measured Moments [Nm]",
                "measured_moment_x",
                "measured_moment_y",
                "measured_moment_z",
                "blue",
            )

        linear_model_plots.plot_covariance_mat(self.X, self.coef_name_list)

        if hasattr(self, "aerodynamics_dict"):
            coef_list = self.optimizer.get_optimization_parameters()
            coef_dict = dict(zip(self.coef_name_list, coef_list))
            aerodynamics_plots.plot_liftdrag_curve(
                self.data_df, coef_dict, self.aerodynamics_dict, self.fisher_metric
            )
        plt.tight_layout()
        plt.show()
        return

    def compute_fisher_information(self):
        ## TODO: Parse accelerometer noise characteristics
        R_acc = np.diag([250 * 0.00186, 250 * 0.00186, 250 * 0.00186])
        R_gyro = np.diag([250 * 0.0003394, 250 * 0.0003394, 250 * 0.0003394])
        ## TODO: Compensate for bandlimited signals
        fudge_factor = 5.0

        self.fisher_metric = {}

        if self.estimate_forces:
            X_forces, y, coef_force = self.assemble_regression_matrices(["lin"])
            X_forces_x = X_forces[0 : self.n_samples, :]
            X_forces_y = X_forces[self.n_samples : 2 * self.n_samples, :]
            X_forces_z = X_forces[2 * self.n_samples : 3 * self.n_samples, :]

            fisher_information_f_mat = np.zeros(shape=(X_forces_x.shape[0], 1))
            fisher_information_f_individual = np.zeros(
                shape=(X_forces_x.shape[0], X_forces_x.shape[1])
            )
            information_matrix_f = np.zeros(
                shape=(X_forces_x.shape[1], X_forces_x.shape[1])
            )
            information_matrix_sum = np.zeros(
                shape=(X_forces_x.shape[1], X_forces_x.shape[1])
            )

            queue_size = 1000
            queue = []

            for i in range(X_forces_x.shape[0]):
                jacobian_f = np.vstack(
                    (X_forces_x[i, :], X_forces_y[i, :], X_forces_z[i, :])
                )
                fisher_information_matrix_f = (
                    np.transpose(jacobian_f) @ np.linalg.inv(R_acc) @ jacobian_f
                )
                information_matrix_f += fisher_information_matrix_f
                queue.append(fisher_information_matrix_f)
                information_matrix_sum += fisher_information_matrix_f
                if len(queue) > queue_size:
                    information_matrix_sum -= queue.pop(0)
                fisher_information_f_mat[i] = min(
                    np.abs(np.linalg.eigvals(information_matrix_sum))
                )
                # fisher_information_f_mat[i]= np.linalg.det(sum(queue))
                # fisher_information_f_mat[i]= np.trace(sum(queue))
                # fisher_information_f_mat[i]= min(np.abs(np.linalg.eigvals(sum(queue)))) / \
                #         max(np.abs(np.linalg.eigvals(sum(queue))))

                fisher_information_f_individual[i, :] = np.diag(
                    fisher_information_matrix_f
                )

            self.data_df[
                ["fisher_information_force"]
            ] = fisher_information_f_mat / np.max(fisher_information_f_mat)
            self.data_df[
                [coef + "_fim" for coef in coef_force]
            ] = fisher_information_f_individual
            try:
                error_covariance_matrix_f = np.linalg.inv(information_matrix_f)
            except np.linalg.LinAlgError:
                warnings.warn(
                    "FIM matrix singular: applying regularization, invalid parameters show Cramer-Rao Bound of 500.0",
                    RuntimeWarning,
                )
                information_matrix_f += 0.0001 * np.eye(information_matrix_f.shape[0])
                error_covariance_matrix_f = np.linalg.inv(information_matrix_f)

            cramer_rao_bounds_f = fudge_factor * np.sqrt(
                np.diag(error_covariance_matrix_f)
            )

            forces_dict = coef_force
            metric_dict = dict(zip(forces_dict, cramer_rao_bounds_f.tolist()))
            print("Cramer-Rao Bounds for force parameters:")
            for key, value in metric_dict.items():
                print(key, "\t", value)

            self.cramer_rao_bounds_f = cramer_rao_bounds_f
            self.fisher_metric.update(metric_dict)

        if self.estimate_moments:
            X_moments, y, coef_moment = self.assemble_regression_matrices(["rot"])
            X_moments_x = X_moments[0 : self.n_samples, :]
            X_moments_y = X_moments[self.n_samples : 2 * self.n_samples, :]
            X_moments_z = X_moments[2 * self.n_samples : 3 * self.n_samples, :]

            fisher_information_m_mat = np.zeros(shape=(X_moments_x.shape[0], 1))
            information_matrix_m = np.zeros(
                shape=(X_moments_x.shape[1], X_moments_x.shape[1])
            )
            fisher_information_m_individual = np.zeros(
                shape=(X_moments_x.shape[0], X_moments_x.shape[1])
            )
            information_matrix_sum = np.zeros(
                shape=(X_moments_x.shape[1], X_moments_x.shape[1])
            )

            queue_size = 1000
            queue = []

            for i in range(X_moments_x.shape[0]):
                jacobian_m = np.vstack(
                    (X_moments_x[i, :], X_moments_y[i, :], X_moments_z[i, :])
                )
                fisher_information_matrix_m = (
                    np.transpose(jacobian_m) @ np.linalg.inv(R_gyro) @ jacobian_m
                )
                information_matrix_m += fisher_information_matrix_m
                queue.append(fisher_information_matrix_m)
                information_matrix_sum += fisher_information_matrix_m
                if len(queue) > queue_size:
                    information_matrix_sum -= queue.pop(0)
                fisher_information_m_mat[i] = min(
                    np.abs(np.linalg.eigvals(information_matrix_sum))
                )
                # fisher_information_m_mat[i]= np.linalg.det(sum(queue))
                # fisher_information_m_mat[i]= np.trace(sum(queue))
                # fisher_information_m_mat[i]= min(np.abs(np.linalg.eigvals(sum(queue)))) / \
                #         max(np.abs(np.linalg.eigvals(sum(queue))))

                fisher_information_m_individual[i, :] = np.diag(
                    fisher_information_matrix_m
                )

            self.data_df["fisher_information_rot"] = fisher_information_m_mat / np.max(
                fisher_information_m_mat
            )
            self.data_df[
                [coef + "_fim" for coef in coef_moment]
            ] = fisher_information_m_individual
            try:
                error_covariance_matrix_m = np.linalg.inv(information_matrix_m)
            except np.linalg.LinAlgError:
                warnings.warn(
                    "FIM matrix singular: applying regularization, invalid parameters show Cramer-Rao Bound of 500.0",
                    RuntimeWarning,
                )
                information_matrix_m += 1e-10 * np.eye(information_matrix_m.shape[0])
                error_covariance_matrix_m = np.linalg.inv(information_matrix_m)

            cramer_rao_bounds_m = fudge_factor * np.sqrt(
                np.diag(error_covariance_matrix_m)
            )

            moments_dict = coef_moment

            metric_dict = dict(zip(moments_dict, cramer_rao_bounds_m.tolist()))
            print("Cramer-Rao Bounds for moment parameters:")
            for key, value in metric_dict.items():
                print(key, "\t", value)

            self.cramer_rao_bounds_m = cramer_rao_bounds_m
            self.fisher_metric.update(metric_dict)

        self.fisher_metric = {"Cramer": self.fisher_metric}

        self.fisher_metric["FIM"] = {}

        if self.estimate_forces:
            self.fisher_metric["FIM"].update(
                {
                    "lin": {
                        "trace": float(np.trace(information_matrix_f) / self.n_samples),
                        "min_eig": float(
                            min(np.abs(np.linalg.eigvals(information_matrix_f)))
                            / self.n_samples
                        ),
                        "inv_cond": float(
                            min(np.abs(np.linalg.eigvals(information_matrix_f)))
                            / max(np.abs(np.linalg.eigvals(information_matrix_f)))
                            / self.n_samples
                        ),
                        "det": float(
                            np.linalg.det(information_matrix_f) / self.n_samples
                        ),
                    }
                }
            )

        if self.estimate_moments:
            self.fisher_metric["FIM"].update(
                {
                    "rot": {
                        "trace": float(np.trace(information_matrix_m)) / self.n_samples,
                        "min_eig": float(
                            min(np.abs(np.linalg.eigvals(information_matrix_m)))
                            / self.n_samples
                        ),
                        "inv_cond": float(
                            min(np.abs(np.linalg.eigvals(information_matrix_m)))
                            / max(np.abs(np.linalg.eigvals(information_matrix_m)))
                            / self.n_samples
                        ),
                        "det": float(
                            np.linalg.det(information_matrix_m) / self.n_samples
                        ),
                    }
                }
            )

            ## min eigenvalue
            # fisher_information_f_mat[i]= min(np.abs(np.linalg.eigvals(fisher_information_matrix_f)))
            # fisher_information_m_mat[i]= min(np.abs(np.linalg.eigvals(fisher_information_matrix_m)))

            ## trace
            # fisher_information_f_mat[i]= np.trace(fisher_information_matrix_f)
            # fisher_information_m_mat[i]= np.trace(fisher_information_matrix_m)

            ## condition number
            # fisher_information_f_mat[i]= min(np.abs(np.linalg.eigvals(fisher_information_matrix_f))) / \
            # max(np.abs(np.linalg.eigvals(fisher_information_matrix_f)))
            # fisher_information_m_mat[i]= min(np.abs(np.linalg.eigvals(fisher_information_matrix_m))) / \
            # max(np.abs(np.linalg.eigvals(fisher_information_matrix_m)))
