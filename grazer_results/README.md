# Grazer reference results

Reference identification runs, kept under version control so later changes have
something to diff against. `model_results/` is gitignored upstream, so anything
worth keeping has to live here.

Log: `System_ID_Flight_2.ulg`, flown 2026-08-31. SDLOG_PROFILE=31, so
`actuator_motors` records at 278 Hz; 178 s airborne with +/-4 m/s translation
and body rates to +/-3.6 rad/s.

| file | config | prefilter |
|---|---|---|
| `System_ID_Flight_2_forces.yaml` | `grazer_quad.yaml` | 0-4 Hz |
| `System_ID_Flight_2_moments.yaml` | `grazer_quad_moments.yaml` | 1.5-4 Hz |

Both align the 32 ms actuator delay. Both use the MEASURED rectangular geometry
(2026-09-30): motor centers form a 0.379 x 0.315 m rectangle, so the position
components are x = 0.1895, y = 0.1575 m. Headline numbers:

| quantity | value | note |
|---|---|---|
| `rot_thrust_quad` | 12.65 | 15.5 N/rotor at u=1; hover check agrees with the measured 1.7 kg |
| `c_m_leaver_quad` | 3.76 | should equal `rot_thrust_quad` by construction -- still 3.4x short |
| `c_m_drag_z_quad` | 0.281 | K_M = 0.0222 m, agreeing with an independent pilot-stick IV estimate (0.0203) |
| `c_m_rolling` | 0.070 | previously railed at zero |
| force R2 | 0.956 | |
| moment R2 | 0.526 | the honest command/response coherence on a closed-loop flight |

## On the geometry

The frame is a RECTANGULAR X: 0.379 m front-to-back between motor centers,
0.315 m left-to-right, so the pitch arm (0.1895) and roll arm (0.1575) differ
by 20%. Earlier runs declared a single shared component of 0.175 m, which is
very nearly the AVERAGE of the two half-spans (0.1735) and therefore reproduced
the correct RADIAL distance to 0.4% -- 0.2475 against the true 0.2464. That is
why the error survived so long: every magnitude check passed. What it got wrong
was the SPLIT, running pitch 8% short and roll 11% long.

The axis assignment comes from the measurement, not from the fit. Motors were
identified individually -- (0,0) back left, (0,.315) back right, (.379,0) front
left, (.379,.315) front right -- so the first coordinate is fore-aft and the
frame is longer than it is wide.

For the record, the three candidate geometries fit like this:

| geometry | `c_m_leaver_quad` | moment R2 |
|---|---|---|
| symmetric 0.175 / 0.175 | 3.57 | 0.492 |
| **rectangular, 0.379 front-back (measured)** | **3.76** | **0.526** |
| rectangular, 0.379 side-to-side | 3.39 | 0.453 |

Do NOT read that ordering as validation. It agrees with the tape measure here,
but that is luck rather than a test: a single shared `c_m_leaver_quad` serves
both the roll and the pitch equation, so the ratio r_y/r_x is the only freedom
the optimizer has to reweight one axis against the other. With a 3.4x
unexplained deficit in that coefficient (below), the arm ratio gets used as a
nuisance parameter that absorbs part of it, and R2 rewards whichever ratio soaks
up the most residual -- not necessarily the true one. Geometry is an input to
this pipeline, not an output of it.

Note this correction did NOT explain the self-consistency gap below: it moved it
from 3.55x to 3.36x, about 5%. The arm error was real but minor.

## On the moment band

The 1.5-4 Hz band was chosen by sweeping, not by taste. `c_m_leaver_quad` is
flat at 3.5-3.8 across highpass 0.8-2.5 Hz and lowpass 4-8 Hz:

| highpass | lowpass | c_m_leaver_quad | R2 |
|---|---|---|---|
| 0.3 | 4.0 | 3.065 | 0.425 |
| 0.5 | 4.0 | 3.350 | 0.461 |
| 0.8 | 4.0 | 3.573 | 0.490 |
| 1.2 | 4.0 | 3.696 | 0.508 |
| **1.5** | **4.0** | **3.764** | **0.526** |
| 2.0 | 4.0 | 3.604 | 0.519 |
| 2.5 | 4.0 | 3.457 | 0.519 |
| 1.2 | 8.0 | 3.316 | 0.582 |
| 2.0 | 8.0 | 3.211 | 0.612 |

Insensitivity to the band is what says the estimate is real rather than an
artifact of the filter. Below ~0.8 Hz it falls away steadily with no plateau, so
that region is contaminated rather than informative. 1.5/4.0 sits in the flat
region.

Widening the lowpass to 8 Hz raises R2 while LOWERING the coefficient. That is
not a better fit in the sense that matters: the extra bandwidth is dominated by
the rate controller's D-term, where command is a function of gyro noise rather
than the reverse, so the added "explained variance" is reverse causality. R2 is
the wrong thing to maximize here.

## Open: c_m_leaver_quad is 3.4x below rot_thrust_quad

Model self-consistency requires the two to be equal -- both are the same rotor
force, one crossed with the arm and one not. 3.76 against 12.65 is a real gap,
and it is now a stable measurement rather than a fitting artifact (it was 0.12
unconditioned, and it no longer moves with the band or with the geometry).

Taken literally it would mean effective arms of 0.047 m (roll) and 0.056 m
(pitch) against geometric 0.1575 and 0.1895, or an inertia 3.4x larger than
measured. Neither is credible. The plausible physical explanation is asymmetric
rotor response: when one rotor speeds up and its opposite slows, the
accelerating rotor is driven by the motor while the decelerating one coasts down
on drag alone, so net DIFFERENTIAL thrust is smaller than twice the commanded
delta even though COLLECTIVE thrust is correct -- which is exactly the pattern
seen here, with `rot_thrust_quad` matching hover weight to 4% while the lever
coefficient falls short.

Confirming that needs rotor speed measured rather than inferred from the
command. This board logs no `esc_status` topic and the airframe has no ESC
telemetry, so it cannot be settled from existing logs.
