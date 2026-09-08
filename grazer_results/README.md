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
| `System_ID_Flight_2_moments.yaml` | `grazer_quad_moments.yaml` | 0.3-4 Hz |

Both align the 32 ms actuator delay. Headline numbers:

| quantity | value | note |
|---|---|---|
| `rot_thrust_quad` | 12.65 | 15.50 N/rotor at u=1; hover check agrees with the measured 1.7 kg |
| `c_m_leaver_quad` | 2.91 | should equal `rot_thrust_quad` by construction -- still 4x short |
| `c_m_drag_z_quad` | 0.202 | K_M = 0.016 m, agreeing with an independent pilot-stick IV estimate (0.020) |
| force R2 | 0.956 | |
| moment R2 | 0.398 | the honest command/response coherence on a closed-loop flight |

`c_m_leaver_quad` remaining 4x below `rot_thrust_quad` is the open item. It is
24x better than the 0.12 the unconditioned fit returned and no longer railing on
a bound, but it is not yet self-consistent.
