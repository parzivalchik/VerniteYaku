package com.verniteyaku.pathing.localization;

import com.verniteyaku.pathing.geometry.ChassisSpeeds;
import com.verniteyaku.pathing.geometry.Pose2d;
import com.verniteyaku.pathing.geometry.Twist2d;

/**
 * Whatever knows where the robot is.
 *
 * <p>In Phase 1 this is a single sensor source -- drive encoders, dead wheels, or
 * an external tracker. In Phase 2 the {@link PoseFuser} sits behind this same
 * interface and fuses several. The follower does not care which, and does not
 * change when the fusion layer lands.
 *
 * <p>All poses are in the start-relative frame described on {@link Pose2d}.
 */
public interface Localizer {

    /**
     * Reads the sensors and advances the estimate. Called once per control loop,
     * before anything asks for the pose.
     */
    void update();

    /** The current best estimate of where the robot is. */
    Pose2d getPose();

    /**
     * Overrides the current estimate. Intended for setting the origin at the
     * start of an auto, not for mid-path corrections -- Phase 2's fusion layer
     * folds vision in as a measurement update instead of a hard reset, precisely
     * so that a bad AprilTag read cannot teleport the robot mid-path.
     */
    void setPose(Pose2d pose);

    /** Robot-relative velocity, inches per second and radians per second. */
    ChassisSpeeds getVelocity();

    /**
     * The robot-relative movement measured over the most recent {@link #update()}.
     * The fuser consumes this as its prediction step.
     */
    default Twist2d getLastTwist() {
        return Twist2d.ZERO;
    }

    /**
     * How much to trust {@link #getPose()}, in [0, 1], where 1 is a tight
     * estimate.
     *
     * <p>A localizer with no notion of its own uncertainty reports 1, which is
     * the honest answer for dead reckoning: it has no idea how wrong it is. Only
     * {@link FusedLocalizer} returns anything else.
     *
     * <p>The follower reads this to decide how hard to correct. Reacting
     * aggressively toward a pose you do not believe is how a robot ends up
     * chasing its own estimation error across the field.
     */
    default double getConfidence() {
        return 1.0;
    }
}
