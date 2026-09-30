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

Both align the 32 ms actuator delay, and both use the geometry measured
2026-09-30: motor centers form a 0.379 (wide) x 0.315 (long) m rectangle, so the
position components are x = 0.1575, y = 0.1895 m. The CG was measured the same
day and sits EXACTLY at the centre of that rectangle in x and y, so all four arms
are equal; rotors are 19 mm below it in z.

| quantity | value | note |
|---|---|---|
| `rot_thrust_quad` | 12.65 | 15.5 N/rotor at u=1; hover at u=0.519 for the measured 1.7 kg |
| `rot_drag_lin` | 0.0833 | vertical drag; the usable translational term |
| `c_m_leaver_quad` | 3.39 | should equal `rot_thrust_quad` by construction -- 3.7x short, OPEN |
| `c_m_drag_z_quad` | 0.2815 | K_M = 0.0222 m, agreeing with an independent pilot-stick IV estimate (0.0203) |
| `c_m_rolling` | 0.0676 | previously railed at zero |
| force R2 | 0.956 | trustworthy |
| moment R2 | 0.453 | the honest command/response coherence on a closed-loop flight |

## Geometry, and why R2 must not be used to choose it

The frame is a RECTANGULAR X, wider than it is long. Motors were measured
individually, origin at the back left motor:

```
back left  (0,     0    )      back right (0.379, 0    )
front left (0,     0.315)      front right(0.379, 0.315)
```

First coordinate is lateral, second is fore-aft, so left-to-right is 0.379 m and
front-to-back is 0.315 m. Halving each gives x = 0.1575 (pitch arm, SHORT) and
y = 0.1895 (roll arm, LONG), radial 0.24641 m. The measurement frame maps onto
FRD with no sign flips.

Earlier runs used a single shared 0.175 m for both components. That number is
almost exactly the AVERAGE of the two half-spans (0.1735), so it reproduced the
correct RADIAL distance to 0.4% -- 0.2475 against the true 0.2464 -- which is why
it went unchallenged for months. It was wrong in the SPLIT, not the size.

**This airframe is the cautionary case for fitting geometry.** The three
candidates score:

| geometry | `c_m_leaver_quad` | moment R2 |
|---|---|---|
| **rectangular, 0.379 lateral (MEASURED)** | **3.39** | **0.453** |
| symmetric 0.175 / 0.175 | 3.57 | 0.492 |
| rectangular, 0.379 fore-aft (wrong way round) | 3.76 | 0.526 |

The measured geometry fits WORST of the three. Do not "improve" it back. One
shared `c_m_leaver_quad` serves both the roll and the pitch equation, so the
ratio r_y/r_x is the optimizer's only freedom to reweight one axis against the
other; with a 3.7x unexplained deficit in that coefficient (below), the arm ratio
gets co-opted as a nuisance parameter absorbing part of it, and R2 then rewards
whichever ratio soaks up the most residual. Geometry is an INPUT here, measured
with a tape. The regression is in no position to identify it while a 3.7x gap
remains unexplained.

## On the moment band

The 1.5-4 Hz band was chosen by sweeping, not by taste. `c_m_leaver_quad` is
flat at 3.1-3.4 across highpass 0.8-2.5 Hz:

| highpass | lowpass | c_m_leaver_quad | R2 |
|---|---|---|---|
| 0.3 | 4.0 | 2.765 | 0.367 |
| 0.5 | 4.0 | 3.000 | 0.395 |
| 0.8 | 4.0 | 3.206 | 0.421 |
| 1.2 | 4.0 | 3.316 | 0.436 |
| **1.5** | **4.0** | **3.386** | **0.453** |
| 2.0 | 4.0 | 3.215 | 0.443 |
| 2.5 | 4.0 | 3.104 | 0.448 |
| 1.2 | 8.0 | 3.027 | 0.519 |
| 2.0 | 8.0 | 2.939 | 0.550 |

Insensitivity to the band is what says the estimate is real rather than an
artifact of the filter. Below ~0.8 Hz it falls away steadily with no plateau, so
that region is contaminated rather than informative.

Widening the lowpass to 8 Hz raises R2 while LOWERING the coefficient. That is
not a better fit in the sense that matters: the extra bandwidth is dominated by
the rate controller's D-term, where command is a function of gyro noise rather
than the reverse, so the added "explained variance" is reverse causality. R2 is
the wrong thing to maximize here, for the same reason it is the wrong way to pick
the geometry.

## Open 1: c_m_leaver_quad is 3.7x below rot_thrust_quad

Model self-consistency requires the two to be equal -- both are the same rotor
force, one crossed with the arm and one not. 3.39 against 12.65 is a real gap,
and it is a stable measurement rather than a fitting artifact: it was 0.12
unconditioned, and it now moves neither with the band nor with the geometry
(3.57 -> 3.76 -> 3.39 across all three candidate geometries).

Taken literally it implies effective arms of 0.051 m (roll) and 0.042 m (pitch)
against geometric 0.1895 and 0.1575, or an inertia 3.7x larger than measured.
Neither is credible.

The leading hypothesis is asymmetric rotor response: when one rotor speeds up and
its opposite slows, the accelerating rotor is driven by the motor while the
decelerating one coasts down on drag alone, so net DIFFERENTIAL thrust is smaller
than twice the commanded delta even though COLLECTIVE thrust is correct. That is
exactly the pattern here -- `rot_thrust_quad` matches hover weight to 4% while the
lever coefficient falls 3.7x short. Confirming it needs rotor speed measured
rather than inferred from the command.

**Status of that measurement.** No `esc_status` is published and the airframe has
no ESC telemetry wired, but the FIRMWARE side is already configured:
`DSHOT_TEL_CFG = 103` (DShot telemetry assigned to TELEM3, whose default is 0 =
disabled, so this was set deliberately), `MOT_POLE_COUNT = 14` for the eRPM->RPM
conversion, and no other `*_CFG` claims TELEM3 (DDS is on TELEM2). So enabling it
is a WIRING job, not a parameter job: one signal wire from the ESC telemetry pad
to TELEM3 RX, if the 4-in-1 exposes that pad.

The wire-free alternative -- bidirectional DShot, which returns eRPM on the signal
line itself -- is NOT available on the flown firmware: `DSHOT_BIDIR_EDT` exists in
the newer upstream tree but not in the mavericks v1.14 fork this board runs. That
route needs a firmware upgrade.

**No PX4 parameter controls the coasting itself.** PX4 only sends a throttle
value; whether a decelerating rotor is actively braked or left to coast down on
prop drag is ESC firmware behaviour (BLHeli_32 / AM32 "Damped Light" /
regenerative braking / complementary PWM). If braking is off, deceleration is set
by prop aerodynamic drag and rotor inertia alone, which is exactly the asymmetry
hypothesised here. Enabling active braking in the ESC would be the fix, not a
PX4 param.

## Separate finding: THR_MDL_FAC = 0.0 is wrong for this airframe

`THR_MDL_FAC = 0.0` tells PX4 that thrust is LINEAR in the throttle command. The
force identification says otherwise and says it cleanly: `rot_thrust_quad` =
12.65 carries essentially everything while `rot_thrust_lin` = -0.062 is
negligible, i.e. thrust is very nearly pure quadratic in the command (R2 0.956).
PX4 is therefore inverting the wrong curve, which makes the effective attitude
loop gain vary with throttle -- high at low collective, low at high collective.

This does NOT explain the lever deficit (a differential pair around hover gives
`c[(u+d)^2 - (u-d)^2] = 4*c*u*d`, which is already linear in d and symmetric), but
it is a real calibration error worth correcting independently. Related board
errors are tracked in the identified-vs-allocator comparison: `CA_ROTOR*_CT` 6.5
against an identified 15.5 N, and `CA_ROTOR*_KM` 0.05 against an identified
0.0222 m.

## CLOSED: the CG is measured, and it is exactly centred

Measured 2026-09-30 relative to the ground under the back left motor, z upward:
CG = (0.1895, 0.1575, 0.1714) m, motor centres 0.1524 m (6 in) above ground.

In x and y that is EXACTLY the centre of the motor rectangle -- offset 0.0 in
both axes. So all four arms are equal, the symmetric-rectangle model is correct
rather than merely assumed, and there is NO static CG trim moment. That closes
the question but also removes a hoped-for lead: the sub-1 Hz content the 1.5 Hz
highpass discards is NOT a CG trim offset, so the highpass cannot be relaxed on
those grounds. Whatever lives down there is something else.

In z, converting to FRD puts the rotors 19 mm BELOW the CG (z = +0.019), where
the config previously said -0.06 -- wrong sign, 3x too large. It turns out not to
matter:

| z | `c_m_leaver_quad` | `c_m_drag_z_quad` | `c_m_rolling` | R2 |
|---|---|---|---|---|
| +0.019 (measured) | 3.3857 | 0.2815 | 0.0676 | 0.4533 |
| 0.0 | 3.3857 | 0.2815 | 0.0644 | 0.4533 |
| -0.06 (old, wrong) | 3.3855 | 0.2815 | 0.0538 | 0.4531 |

Only `c_m_rolling` responds, and R2 does not move. The lever term is structurally
immune: `cross(r, [0,0,-1]) = (-r_y, r_x, 0)`, so z cancels exactly.

NB 0.1524 m is the MOTOR CENTRE height, but thrust acts at the PROP DISC some
15-25 mm higher, which would put the thrust plane level with the CG (z ~ 0). The
honest value is somewhere in 0.000 to +0.019 -- the offset is smaller than its own
uncertainty. Given the table above, this does not matter.
