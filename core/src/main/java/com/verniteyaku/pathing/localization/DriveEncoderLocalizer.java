package com.verniteyaku.pathing.localization;

import com.verniteyaku.pathing.control.Clock;
import com.verniteyaku.pathing.drive.Drivetrain;
import com.verniteyaku.pathing.geometry.Angles;
import com.verniteyaku.pathing.geometry.ChassisSpeeds;
import com.verniteyaku.pathing.geometry.Pose2d;
import com.verniteyaku.pathing.geometry.Twist2d;
import com.verniteyaku.pathing.kinematics.Kinematics;

/**
 * Odometry from the drive motors' own encoders, optionally with the IMU
 * supplying heading.
 *
 * <p>This is the baseline localizer: it needs no dead wheels and no extra
 * hardware. It is also the least accurate one, because mecanum wheels slip
 * laterally and the encoders cannot see it -- expect drift to accumulate over a
 * long auto. Supplying a {@link HeadingSource} fixes the worst of it, since
 * heading error is what turns small translation errors into large ones.
 *
 * <p>Phase 2's EKF replaces the heading override here with a proper measurement
 * update, so the IMU informs the estimate in proportion to its trustworthiness
 * rather than simply overwriting it.
 */
public final class DriveEncoderLocalizer implements Localizer {

    private final Drivetrain drivetrain;
    private final Kinematics kinematics;
    private final Clock clock;
    private final HeadingSource headingSource;

    private Pose2d pose = Pose2d.ZERO;
    private double[] lastWheelPositions;
    private double lastTime = Double.NaN;
    private double headingOffset = Double.NaN;
    private ChassisSpeeds velocity = ChassisSpeeds.ZERO;
    private Twist2d lastTwist = Twist2d.ZERO;

    /** Encoder-only odometry. Heading is integrated from the wheels. */
    public DriveEncoderLocalizer(Drivetrain drivetrain, Clock clock) {
        this(drivetrain, clock, null);
    }

    /**
     * Encoder odometry with heading taken from {@code headingSource}.
     *
     * @param headingSource may be null, in which case heading is integrated from
     *                      the wheels alone
     */
    public DriveEncoderLocalizer(Drivetrain drivetrain, Clock clock,
                                 HeadingSource headingSource) {
        if (drivetrain == null || clock == null) {
            throw new IllegalArgumentException("drivetrain and clock must be non-null");
        }
        this.drivetrain = drivetrain;
        this.kinematics = drivetrain.getKinematics();
        this.clock = clock;
        this.headingSource = headingSource;
    }

    @Override
    public void update() {
        double now = clock.seconds();
        double[] positions = drivetrain.getWheelPositions();

        if (lastWheelPositions == null) {
            lastWheelPositions = positions.clone();
            lastTime = now;
            if (headingSource != null) {
                // Whatever the gyro reads at the first update defines heading
                // zero, which is what makes the frame start-relative.
                headingOffset = headingSource.getHeadingRadians() - pose.heading;
            }
            return;
        }

        double[] deltas = new double[positions.length];
        for (int i = 0; i < positions.length; i++) {
            deltas[i] = positions[i] - lastWheelPositions[i];
        }
        lastWheelPositions = positions.clone();

        Twist2d twist = kinematics.toTwist(deltas);

        if (headingSource != null) {
            // Trust the gyro over the wheels for rotation: wheel-derived heading
            // drifts fastest of anything here.
            double measured = Angles.normalize(headingSource.getHeadingRadians() - headingOffset);
            double gyroDelta = Angles.normalize(measured - pose.heading);
            twist = new Twist2d(twist.dx, twist.dy, gyroDelta);
        }

        lastTwist = twist;
        pose = twist.applyTo(pose);

        double dt = now - lastTime;
        lastTime = now;
        if (dt > 1e-9) {
            velocity = new ChassisSpeeds(twist.dx / dt, twist.dy / dt, twist.dTheta / dt);
        }
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void setPose(Pose2d pose) {
        if (pose == null) {
            throw new IllegalArgumentException("pose must be non-null");
        }
        this.pose = pose;
        if (headingSource != null) {
            headingOffset = headingSource.getHeadingRadians() - pose.heading;
        }
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    @Override
    public Twist2d getLastTwist() {
        return lastTwist;
    }
}
