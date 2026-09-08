# Migrating

## From Pedro Pathing

The path-building API is deliberately close, so most autos port with small edits.

### What is the same

```java
// Pedro
PathChain chain = follower.pathBuilder()
        .addPath(new BezierLine(new Point(0, 0), new Point(24, 0)))
        .setLinearHeadingInterpolation(0, Math.toRadians(90))
        .build();

// VerniteYaku
PathChain chain = new PathBuilder()
        .addPath(new BezierLine(new Point(0, 0), new Point(24, 0)))
        .setLinearHeadingInterpolation(0, Math.toRadians(90))
        .build();
```

`addPath`, `setLinearHeadingInterpolation`, `setConstantHeadingInterpolation`,
`setTangentHeadingInterpolation`, `BezierLine`, `BezierCurve`, `Point`,
`PathChain` all behave as you expect.

The follow loop is the same shape:

```java
follower.followPath(chain);
while (opModeIsActive() && follower.isBusy()) {
    follower.update();
}
```

### What changes

| Pedro | Here |
|---|---|
| `follower.pathBuilder()` | `new PathBuilder()` — not owned by the follower |
| Field-relative coordinates | **Start-relative.** See below |
| `Point(x, y, Point.CARTESIAN)` | `new Point(x, y)`, or `Point.of(x, y, unit)` |
| Heading in `Math.toRadians(...)` | Same — radians throughout |
| `follower.setStartingPose(...)` | `localizer.setPose(...)` |
| `follower.holdPoint(...)` | Not built. Use a one-point path or hold position yourself |
| `FollowerConstants` static fields | An immutable builder, per-follower |

**The coordinate frame is the big one.** Pedro uses field coordinates; here the
origin is wherever the robot starts. A Pedro path that begins at field
`(9, 60, 0)` becomes a path beginning at `(0, 0, 0)`, with every subsequent point
expressed relative to that start.

To port a path, subtract the start position from every point and rotate by the
negative start heading:

```java
// If the Pedro auto started at field (9, 60) facing 0 degrees, then a Pedro
// point (33, 60) becomes (33-9, 60-60) = (24, 0) here.
```

If your start heading was not zero, rotate as well — or let
`Pose2d.relativeTo(startPose)` do it.

Tuning constants do **not** carry over. Pedro's are structured differently; start
from [tuning.md](tuning.md) and re-tune. The [online tuner](tuning.md#online-tuning)
will fit `kS`/`kV`/`kA` for you after a couple of runs.

---

## From Road Runner

### What is familiar

Time-parameterised motion profiling with feedforward is the same idea, and
`kS`/`kV`/`kA` mean the same thing — with one difference noted below. Paths take
the same amount of time every run, so an auto timed around them stays timed.

### What changes

| Road Runner | Here |
|---|---|
| `TrajectorySequenceBuilder` | `PathBuilder` — geometry only, no time |
| `Actions.runBlocking(...)` | Your own loop, or the [command layer](commands.md) |
| Field coordinates | Start-relative |
| `kV` against raw duty | `kV` against **battery-normalised** voltage |
| Trajectory-level markers | Not built. Sequence with commands instead |
| `MecanumDrive` doing everything | `Drivetrain` + `Kinematics` + `Localizer` + `Follower` |

**The `kV` difference matters.** Road Runner's `kV` maps velocity to a raw duty
cycle. Here it maps velocity to a fraction of *nominal 12 V*, so the drivetrain
divides by the actual battery voltage before writing. A duty of 0.5 means
something different at 13 V than at 11 V; normalising means the model does not
silently re-fit itself as the battery drains.

If you are porting a Road Runner `kV`, it will be roughly right at a full
battery and increasingly wrong as it sags. Re-tune, or let the online tuner do it.

Road Runner actions drive this follower directly — see
[commands.md](commands.md#using-a-different-framework) for the adapter.

---

## What this library does not have

Be aware before committing to a port:

- **No AprilTag integration.** `VisionPoseSource` is a tested interface with
  nothing behind it.
- **No tank or swerve.** The `Kinematics` interface allows them; only mecanum is
  implemented.
- **No holdPoint / turnTo primitives.** Express them as short paths.
- **No trajectory markers.** Sequence with the command layer instead.
- **No dashboard integration.** Telemetry is yours to wire up.
- **Not yet run on real hardware.** See [BRINGUP.md](../BRINGUP.md).

---

## Why port at all

The things neither of the above offers:

- **Fused localization.** An EKF over encoders and IMU, with vision entering as a
  weighted measurement rather than a `resetPose()` that teleports the robot
  mid-path.
- **Hybrid correction.** Road Runner's time-consistency and Pedro's aggressive
  recovery from one follower, blended on error magnitude rather than switched.
- **Online auto-tuning.** `kS`/`kV`/`kA` fit themselves while you drive, and
  persist between OpMode runs.
- **Stall detection** against the model's predicted current, not a fixed
  threshold.
- **Collision planning.** A browser tool that checks two alliance partners' autos
  against each other and exports code in this library's own API.
