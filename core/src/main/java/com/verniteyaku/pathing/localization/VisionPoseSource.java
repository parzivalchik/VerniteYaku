package com.verniteyaku.pathing.localization;

import com.verniteyaku.pathing.geometry.Pose2d;

/**
 * A source of absolute pose observations -- in practice, AprilTags.
 *
 * <p><b>STUB. Nothing implements this yet.</b> The interface and the plumbing in
 * {@link FusedLocalizer} that consumes it are finished and tested against
 * scripted observations; what is deliberately not built is the AprilTag half --
 * camera calibration, tag-field layout, and turning a detection into a
 * start-relative pose. That is Phase 3 work and is listed as such in the README.
 *
 * <p>It exists now because the shape of this interface constrains the filter's
 * design, and getting that wrong would mean rewriting the fusion layer later. In
 * particular, an observation must carry its own uncertainty: a tag seen head-on
 * at two feet and the same tag glimpsed at the frame's edge at ten feet are not
 * remotely the same measurement, and a fusion layer that cannot tell them apart
 * has no way to behave sensibly.
 *
 * <h2>Implementing this later</h2>
 * <ul>
 *   <li>Return {@code null} from {@link #getObservation()} whenever there is no
 *       fresh detection. The localizer skips the update; it never blocks.</li>
 *   <li>Report the pose in the <b>start-relative</b> frame, not the field frame.
 *       That means the implementation has to know where the robot started, which
 *       is a decision for the OpMode, not for this library.</li>
 *   <li>Scale the variances with observed range and viewing angle. Constant
 *       variances would defeat the purpose of fusing at all.</li>
 * </ul>
 */
public interface VisionPoseSource {

    /** One absolute pose observation and how much to trust it. */
    final class Observation {
        /** The observed pose, start-relative. */
        public final Pose2d pose;
        /** {x, y, heading} variances: inches squared and radians squared. */
        public final double[] variance;
        /** When the observation was taken, seconds on the follower's clock. */
        public final double timestamp;

        public Observation(Pose2d pose, double[] variance, double timestamp) {
            if (pose == null) {
                throw new IllegalArgumentException("pose must be non-null");
            }
            if (variance == null || variance.length != 3) {
                throw new IllegalArgumentException("variance must be {x, y, heading}");
            }
            this.pose = pose;
            this.variance = variance.clone();
            this.timestamp = timestamp;
        }
    }

    /**
     * The latest observation, or {@code null} if there is nothing new this loop.
     * Must not block: it is called from the control loop.
     */
    Observation getObservation();
}
