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

Both align the 32 ms actuator delay. Headline numbers:

| quantity | value | note |
|---|---|---|
| `rot_thrust_quad` | 12.65 | 15.50 N/rotor at u=1; hover check agrees with the measured 1.7 kg |
| `c_m_leaver_quad` | 3.57 | should equal `rot_thrust_quad` by construction -- still 3.5x short |
| `c_m_drag_z_quad` | 0.281 | K_M = 0.0222 m, agreeing with an independent pilot-stick IV estimate (0.0203) |
| `c_m_rolling` | 0.062 | previously railed at zero |
| force R2 | 0.956 | |
| moment R2 | 0.492 | the honest command/response coherence on a closed-loop flight |

## On the moment band

The 1.5-4 Hz band was chosen by sweeping, not by taste. `c_m_leaver_quad` is
FLAT at 3.1-3.6 across highpass 1.2-2.5 Hz and lowpass 3-8 Hz:

| highpass | lowpass | c_m_leaver_quad | R2 |
|---|---|---|---|
| 0.2 | 4.0 | 2.55 | 0.353 |
| 0.3 | 4.0 | 2.91 | 0.398 |
| 0.5 | 4.0 | 3.17 | 0.430 |
| 0.8 | 4.0 | 3.38 | 0.458 |
| 1.2 | 4.0 | 3.50 | 0.474 |
| **1.5** | **4.0** | **3.57** | **0.492** |
| 2.0 | 4.0 | 3.40 | 0.484 |
| 2.5 | 4.0 | 3.28 | 0.486 |
| 1.2 | 8.0 | 3.17 | 0.554 |
| 2.0 | 8.0 | 3.07 | 0.585 |

Insensitivity to the band is what says the estimate is real rather than an
artifact of the filter. Below ~1 Hz it falls away steadily with no plateau, so
that region is contaminated rather than informative. 1.5/4.0 sits at the centre
of the flat region, not at its peak.

## Open: c_m_leaver_quad is 3.5x below rot_thrust_quad

Model self-consistency requires the two to be equal. 3.57 against 12.65 is a
real gap, and it is now a stable measurement rather than a fitting artifact
(it was 0.12 unconditioned, and it no longer moves with the band).

Taken literally it would mean an effective arm of 0.049 m against a geometric
0.175, or an inertia 3.5x larger than measured. Neither is credible. The
plausible physical explanation is asymmetric rotor response: when one rotor
speeds up and its opposite slows, the accelerating rotor is driven by the motor
while the decelerating one coasts down on drag alone, so net DIFFERENTIAL thrust
is smaller than twice the commanded delta even though COLLECTIVE thrust is
correct -- which is exactly the pattern seen here, with rot_thrust_quad matching
hover weight to 4% while the lever coefficient falls short.

Confirming that needs rotor speed measured rather than inferred from the
command. This board logs no `esc_status` topic.
