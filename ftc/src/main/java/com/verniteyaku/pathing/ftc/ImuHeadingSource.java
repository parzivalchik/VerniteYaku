package com.verniteyaku.pathing.ftc;

import com.qualcomm.robotcore.hardware.IMU;
import com.verniteyaku.pathing.localization.HeadingSource;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Adapts the Control Hub's IMU to {@link HeadingSource}.
 *
 * <p>The IMU must already be initialised with the correct hub orientation before
 * this is constructed -- the orientation depends on how the hub is bolted to the
 * robot, which the library cannot know.
 *
 * <p>Reads are unwrapped internally. The SDK reports yaw in (-180, 180]; handing
 * that straight to the localizer would make a robot spinning past 180 degrees
 * appear to snap a full turn backwards in a single loop. Accumulating the
 * differences instead gives a continuous heading.
 */
public final class ImuHeadingSource implements HeadingSource {

    private final IMU imu;
    private double continuousHeading;
    private double lastRawHeading;
    private boolean initialised;

    public ImuHeadingSource(IMU imu) {
        if (imu == null) {
            throw new IllegalArgumentException("imu must be non-null");
        }
        this.imu = imu;
    }

    @Override
    public double getHeadingRadians() {
        double raw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (!initialised) {
            continuousHeading = raw;
            lastRawHeading = raw;
            initialised = true;
            return continuousHeading;
        }

        double delta = raw - lastRawHeading;
        // A jump of more than half a turn between two reads is a wrap, not real
        // motion -- no FTC robot rotates 180 degrees inside one control loop.
        if (delta > Math.PI) {
            delta -= 2 * Math.PI;
        } else if (delta < -Math.PI) {
            delta += 2 * Math.PI;
        }

        continuousHeading += delta;
        lastRawHeading = raw;
        return continuousHeading;
    }

    /** Zeroes the underlying IMU's yaw and restarts unwrapping from there. */
    public void resetYaw() {
        imu.resetYaw();
        initialised = false;
    }
}
