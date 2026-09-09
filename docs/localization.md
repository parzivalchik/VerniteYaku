# Localization

Two implementations behind one interface. The follower cannot tell them apart, so
swapping is a one-line change.

---

## Localizer

```java
public interface Localizer {
    void update();                  // read sensors, advance the estimate
    Pose2d getPose();               // best estimate, field coordinates
    void setPose(Pose2d pose);      // override -- prefer startPose at init
    ChassisSpeeds getVelocity();    // robot-relative
    Twist2d getLastTwist();         // movement over the last update
    double getConfidence();         // [0, 1], 1 = tight estimate
}
```

`getConfidence()` defaults to 1.0. That is the honest answer for dead reckoning:
it has no idea how wrong it is. Only `FusedLocalizer` returns anything else.

**Both need a `startPose`.** Poses are absolute field coordinates, and absolute
coordinates cannot be inferred from encoders. The default is the field centre
facing +X, which is almost certainly not where your robot is — get it wrong and
every path is offset by the same amount.

---

## DriveEncoderLocalizer

Odometry from the drive motors' own encoders, with the IMU optionally overriding
wheel-derived heading.

```java
DriveEncoderLocalizer localizer = DriveEncoderLocalizer.builder(drivetrain, Clock.system())
        .startPose(new Pose2d(-60, -36, 0))
        .headingSource(new ImuHeadingSource(imu))
        .build();
```

No extra hardware, and the least accurate option: mecanum wheels slip laterally
and the encoders cannot see it, so drift accumulates over a long auto. Supplying
a `HeadingSource` fixes the worst of it, because heading error is what turns
small translation errors into large ones.

Start here during bring-up. It has fewer moving parts, so when the pose is wrong
you know it is the sensors and not the filter.

---

## FusedLocalizer

An extended Kalman filter over `[x, y, heading]`. Odometry is the prediction
step; the IMU — and later AprilTags — enter as **measurement updates** weighted
by their own variance.

```java
FusedLocalizer localizer = FusedLocalizer.builder(drivetrain, Clock.system())
        .startPose(new Pose2d(-60, -36, 0))
        .headingSource(new ImuHeadingSource(imu))
        .headingVariance(1e-4)     // ~0.6 degrees of sigma
        .build();
```

### Why a filter instead of resetPose()

The common FTC approach is to call `resetPose()` when a tag comes into view. That
discards a good odometry estimate in favour of a single frame's observation, and
teleports the robot mid-path when that observation is marginal.

Here a measurement moves the estimate in proportion to how much it deserves to.
Measured over 60 repeated fixes from the same starting error:

| Observation variance | Gap closed |
|---|---|
| 0.25 in² (vague) | 67% |
| 0.05 in² | 84% |
| 0.01 in² (tight) | 96% |

A `resetPose()` override closes 100% every time, regardless of whether the
observation earned it.

### Process noise knows what odometry is bad at

Noise is built in the robot's own frame — where "forward" and "sideways" have
genuinely different error characteristics — then rotated into the world.
`lateralNoisePerInch` defaults five times higher than the forward figure, because
mecanum scrubs sideways and the encoders cannot see it.

Building noise directly in world coordinates would smear the good along-track
estimate into the bad lateral one and lose exactly the information that makes
filtering worth doing.

A useful consequence: because the covariance carries correlations, a **heading**
measurement corrects **position** too. A robot that drove a curve has correlated
position and heading error, so learning the heading says something about where it
is. A `resetPose()`-style override throws that away.

| Setting | Default | Meaning |
|---|---|---|
| `translationalNoisePerInch` | 0.002 | Variance added per inch of forward travel, in² |
| `lateralNoisePerInch` | 0.010 | Same, sideways. Deliberately larger |
| `headingNoisePerRadian` | 0.020 | Variance per radian of rotation, rad² |
| `baseNoisePerSecond` | 1e-6 | Added per second regardless of motion |
| `confidenceScale` | 2.0 | Position sigma, inches, at which confidence reads ~0.5 |

`baseNoisePerSecond` keeps the covariance from collapsing to zero while the robot
sits still, which would make the filter ignore every subsequent measurement.

### Confidence

`getConfidence()` is a bounded summary of positional uncertainty, in [0, 1]. It
feeds the follower's correction authority: reacting aggressively toward a pose
you do not believe is how a robot ends up chasing its own estimation error across
the field. Floored by `minConfidenceAuthority` so a doubted estimate still
corrects somewhat.

---

## HeadingSource

```java
@FunctionalInterface
public interface HeadingSource {
    double getHeadingRadians();
}
```

`ImuHeadingSource` wraps the Control Hub IMU and **unwraps** its reading
internally. The SDK reports yaw in (-180°, 180°]; handing that straight to the
localizer makes a robot spinning past 180° appear to snap a full turn backwards
in one loop. Differences are accumulated instead, giving a continuous heading.

The IMU must already be initialised with your hub's orientation before this is
constructed — the library takes it as it finds it, because the orientation
depends on how the hub is bolted to the robot.

Whatever the gyro reads at the first `update()` is calibrated against the
configured `startPose` heading. That is what puts the estimate in field
coordinates rather than relative to power-on — the IMU's own zero is arbitrary
and never appears in a pose.

---

## Vision

`VisionPoseSource` is a **stub**. The interface and the `FusedLocalizer` plumbing
that consumes it are finished and tested against scripted observations. The
AprilTag half — camera calibration, tag-field layout, turning a detection into a
field-coordinate pose — is not built.

```java
public interface VisionPoseSource {
    final class Observation {
        public final Pose2d pose;        // field coordinates
        public final double[] variance;  // {x, y, heading}
        public final double timestamp;
    }
    Observation getObservation();        // null when nothing is fresh
}
```

It exists now because the shape of this interface constrains the filter's design,
and getting it wrong would mean rewriting the fusion layer later. In particular,
**an observation must carry its own uncertainty**: a tag seen head-on at two feet
and the same tag glimpsed at the frame's edge at ten feet are not the same
measurement, and a fusion layer that cannot tell them apart has no way to behave
sensibly.

If you implement it:

- Return `null` when there is no fresh detection. The localizer skips the update;
  it never blocks.
- Report poses in **field coordinates**. This is the easy direction now: a tag's
  field position is fixed and published, so a detection converts straight into an
  absolute pose with no knowledge of where the robot started.
- Scale variances with observed range and viewing angle. Constant variances
  defeat the purpose of fusing.

Stale observations are rejected by timestamp. A camera pipeline runs slower than
the control loop and re-serves the same detection for several loops; applying it
repeatedly would let one measurement shrink the covariance over and over and
convince the filter it is far more certain than it has any right to be.
