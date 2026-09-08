# Bring-up: getting VerniteYaku onto a real robot

**None of this library has run on hardware.** 233 tests pass, but they are
headless and the simulation is kinematic — no motor dynamics, no battery sag, no
wheel slip. They prove the geometry, the control structure and the math converge.
They do not prove any gain is right for your robot.

Work through this in order. Each step is checkable on its own, and each one
catches a failure that is nearly impossible to diagnose once you have stacked
three of them together.

Budget an afternoon. Do it on blocks first — a robot with a wrong motor
reversal at full path speed hits a wall.

---

## 0. Before you power on

- [ ] Robot on blocks, wheels off the ground.
- [ ] Battery charged above 13 V. A sagging battery makes every measurement
      below wrong, and you will not know which ones.
- [ ] Motor names in your robot configuration match the strings in your OpMode
      exactly. `frontLeft` is not `front_left`.
- [ ] `local.properties` has your `sdk.dir`, or `ANDROID_HOME` is set, so
      `./gradlew :ftc:assembleDebug` works.

---

## 1. Motor direction

**The single most common cause of a brand-new robot doing something baffling.**

Write a scratch OpMode that runs one motor at a time at 0.3 power for a second.

- [ ] Each named motor is the wheel you think it is.
- [ ] Each wheel, run alone and positive, drives that corner of the robot
      **forward**. Flip the corresponding flag in `.reverse(fl, fr, bl, br)`
      until all four do.

Then, still on blocks, command all four positive together.

- [ ] All four wheels spin forward. If the robot would spin instead, one side is
      still mirrored.

Now on the floor, drive `setWheelPowers(new double[]{0.3, 0.3, 0.3, 0.3})`.

- [ ] Robot drives **forward**, not sideways, not in a circle.

And a strafe, `{-0.3, 0.3, 0.3, -0.3}`:

- [ ] Robot strafes **left**. If it strafes right, your roller pattern is
      mirrored from the assumed X configuration; swap the front and back pairs.

> Everything downstream assumes +x is forward and +y is left. Nothing else in
> this checklist will make sense until this step is right.

---

## 2. Encoders and geometry

- [ ] `ticksPerRevolution` matches your motor's datasheet **at the output
      shaft**, after the gearbox. A 312 RPM goBILDA Yellow Jacket is 537.7, not
      the bare-motor count.
- [ ] `wheelRadius` is the radius, not the diameter. A 96 mm wheel is 1.89 in.
- [ ] `maxMotorRpm` is the free speed from the datasheet.

Push the robot **by hand** exactly 24 inches forward against a tape measure,
printing `drivetrain.getWheelPositions()`.

- [ ] All four readings are close to +24, and none is negative. A negative one
      means that encoder is reversed relative to its motor.
- [ ] If they read consistently 24 × *k*, your `ticksPerRevolution` or
      `wheelRadius` is off by *k*. Fix the number; do not add a fudge factor.

Then `trackWidth` and `wheelBase`: measure wheel **centre to centre**, not the
robot's outside dimensions.

---

## 3. IMU

- [ ] The IMU is initialised with your hub's actual orientation before
      `ImuHeadingSource` is constructed. The library takes it as it finds it.
- [ ] Print `heading.getHeadingRadians()` and rotate the robot 90° CCW by hand:
      the value increases by about π/2. If it decreases, your hub orientation is
      wrong.
- [ ] Spin the robot slowly through more than a full turn. The value keeps
      climbing past π rather than jumping — that is the unwrapping working. A
      jump here means something is wrapping the value before it reaches the
      library.

---

## 4. Localizer

Run a `DriveEncoderLocalizer` (start simple; add fusion after) and print the
pose while you push the robot by hand.

- [ ] Push forward 24 in → pose reads roughly (24, 0, 0).
- [ ] Push left 24 in → roughly (0, 24, 0).
- [ ] Rotate 90° CCW in place → heading ≈ π/2, position barely moves.
- [ ] Push in a closed square back to the start → pose returns to roughly
      (0, 0, 0). Expect a few inches of drift; expect much more if you skipped
      the IMU.

Only once that is clean, switch to `FusedLocalizer` and repeat. It should be at
least as good. If it is worse, your `headingVariance` is wrong.

---

## 5. Feedforward

Tune this **before** touching PID. If `kV` is wrong, the PID spends the whole
path papering over a systematic error and no gain will look right.

Start from `kV = 1 / maxWheelVelocity` (in/s) and `kS = 0`, `kA = 0`.

**kV** — command a constant power, measure the steady wheel speed:

- [ ] At 0.5 power the robot reaches roughly half its top speed.
- [ ] Adjust `kV` until `kV × measured_velocity ≈ commanded_power` across
      several speeds. Use the middle of the range, not the extremes.

**kS** — raise from 0 until the robot *just barely* creeps from rest. That is the
value; do not go past it.

**kA** — leave at 0 until velocity tracking is good. It only matters during hard
acceleration and is the easiest of the three to make things worse with.

> Once the online tuner has run for a match it will fit all three for you, and
> `FtcTuningStore` will remember them. This step is about getting close enough
> that the first auto is safe.

---

## 6. First path — slow

Set `maxVelocity` to about **a third** of what you eventually want. A tuning
error at 15 in/s is a bad demo; the same error at 45 in/s is a broken robot.

Start with a 24-inch straight line, tangent heading.

- [ ] Robot drives roughly straight and stops near the target.
- [ ] `follower.isAtTarget()` returns true. If it is false, it timed out short —
      that is a miss, not a success, regardless of where the robot ended up.

Put these on telemetry for every run from here on:

| Value | What it tells you |
|---|---|
| `getPose()` | Where the library thinks it is |
| `getPositionError()` | How far off the profile's setpoint |
| `getCorrectionAuthority()` | 1.0 = tracking fine; rising = fighting back |
| `getPoseConfidence()` | How much the localizer trusts itself |

---

## 7. PID

Raise `translationalPID` kP until the robot corrects briskly without
oscillating; then `headingPID` kP the same way.

- [ ] Shove the robot mid-path. It comes back to the line and does not weave
      after recovering.
- [ ] `getCorrectionAuthority()` sits near 1.0 while tracking and rises toward
      your `reactiveAuthority` when shoved. If it sits high all the time, your
      `reactiveErrorScale` is too small for how well the robot actually tracks.

Only now raise `maxVelocity` toward the real value, in steps, re-checking after
each.

---

## 8. Curves, chains, and the rest

- [ ] A single Bezier curve with tangent heading.
- [ ] A curve with `setConstantHeadingInterpolation` — the robot should hold its
      heading while the path turns under it.
- [ ] A two-segment chain. Watch that it does not stop at the junction.
- [ ] A segment with `setMaxVelocity` — the robot should already be slowed
      *before* it reaches the capped segment, not braking at the boundary.

---

## 9. Stall detection

- [ ] With the robot driving normally, `expectedCurrent` against measured is
      within about 2×. If healthy driving already reads 3× over, your
      `stallCurrentAmps` is wrong for your motor.
- [ ] Hold the robot against a wall mid-path. A stall fires within your debounce
      window.
- [ ] Accelerate hard from rest. **No** stall fires — this is the false positive
      a fixed current threshold would give you.

Remember the library takes no action on a stall. Decide what yours should do.

---

## When something is wrong

| Symptom | Look at first |
|---|---|
| Spins instead of driving forward | Motor reversal (§1) |
| Strafes the wrong way | Roller pattern / front-back pair swap (§1) |
| Drives right distance, wrong scale | `ticksPerRevolution`, `wheelRadius` (§2) |
| Pose heading drifts fast | IMU orientation, or no `HeadingSource` (§3, §4) |
| Tracks fine slow, drifts off fast | `kV` too low — wheels saturating (§5) |
| Never reaches the target, times out | `kS` too low, or `positionTolerance` too tight |
| Oscillates around the path | `translationalPID` kP too high (§7) |
| Corrects violently on small errors | `reactiveErrorScale` too small (§7) |
| Auto differs run to run | Something is changing gains mid-run — check that
  nothing calls `setActiveImmediately` outside init |

---

## What is still unbuilt

- **AprilTags.** `VisionPoseSource` is a tested interface with nothing behind it.
  The `FusedLocalizer` will fold observations in correctly the moment something
  produces them.
- **Tank and swerve.** The `Kinematics` interface has room; only mecanum exists.
- **MPC.** Out of scope; the error-scaled reactive term is the design point.
