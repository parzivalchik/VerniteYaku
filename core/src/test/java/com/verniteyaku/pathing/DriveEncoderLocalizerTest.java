package com.verniteyaku.pathing;

import com.verniteyaku.pathing.drive.Drivetrain;
import com.verniteyaku.pathing.geometry.Pose2d;
import com.verniteyaku.pathing.kinematics.Kinematics;
import com.verniteyaku.pathing.kinematics.MecanumKinematics;
import com.verniteyaku.pathing.localization.DriveEncoderLocalizer;
import com.verniteyaku.pathing.sim.SimulatedRobot;
import com.verniteyaku.pathing.units.DistanceUnit;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DriveEncoderLocalizerTest {

    private MecanumKinematics kinematics;
    private SimulatedRobot.ManualClock clock;
    private FakeEncoders encoders;

    @BeforeEach
    void setUp() {
        kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60)
                .build();
        clock = new SimulatedRobot.ManualClock();
        encoders = new FakeEncoders(kinematics);
    }

    /** A drivetrain whose encoders the test drives directly. */
    private static final class FakeEncoders implements Drivetrain {
        private final Kinematics kinematics;
        private final double[] positions;

        FakeEncoders(Kinematics kinematics) {
            this.kinematics = kinematics;
            this.positions = new double[kinematics.getWheelCount()];
        }

        /** Adds the wheel travel corresponding to one wheel-velocity command. */
        void advance(double[] wheelDeltas) {
            for (int i = 0; i < positions.length; i++) {
                positions[i] += wheelDeltas[i];
            }
        }

        @Override
        public Kinematics getKinematics() {
            return kinematics;
        }

        @Override
        public void setWheelPowers(double[] powers) {
            // not used by the localizer
        }

        @Override
        public double[] getWheelPositions() {
            return positions.clone();
        }
    }

    @Test
    void firstUpdateOnlyEstablishesABaselineAndDoesNotMove() {
        DriveEncoderLocalizer localizer = new DriveEncoderLocalizer(encoders, clock);
        encoders.advance(new double[]{100, 100, 100, 100});
        localizer.update();

        assertEquals(0.0, localizer.getPose().getX(), 1e-9,
                "the first read defines the origin; it must not be integrated");
    }

    @Test
    void straightDrivingIntegratesForward() {
        DriveEncoderLocalizer localizer = new DriveEncoderLocalizer(encoders, clock);
        localizer.update();

        for (int i = 0; i < 10; i++) {
            encoders.advance(new double[]{1, 1, 1, 1});
            clock.advance(0.02);
            localizer.update();
        }

        assertEquals(10.0, localizer.getPose().getX(), 1e-9);
        assertEquals(0.0, localizer.getPose().getY(), 1e-9);
        assertEquals(0.0, localizer.getPose().getHeading(), 1e-9);
    }

    @Test
    void strafingIntegratesSideways() {
        DriveEncoderLocalizer localizer = new DriveEncoderLocalizer(encoders, clock);
        localizer.update();

        for (int i = 0; i < 10; i++) {
            encoders.advance(new double[]{-1, 1, 1, -1});
            clock.advance(0.02);
            localizer.update();
        }

        assertEquals(0.0, localizer.getPose().getX(), 1e-9);
        assertEquals(10.0, localizer.getPose().getY(), 1e-9);
    }

    @Test
    void velocityIsReportedFromTheTimestep() {
        DriveEncoderLocalizer localizer = new DriveEncoderLocalizer(encoders, clock);
        localizer.update();

        encoders.advance(new double[]{0.2, 0.2, 0.2, 0.2});
        clock.advance(0.02);
        localizer.update();

        assertEquals(10.0, localizer.getVelocity().vx, 1e-9); // 0.2" in 0.02s
    }

    @Test
    void headingSourceOverridesWheelDerivedRotation() {
        // The wheels claim a pure rotation; the gyro says the robot never turned.
        // The gyro must win.
        double[] gyro = {0.0};
        DriveEncoderLocalizer localizer =
                new DriveEncoderLocalizer(encoders, clock, () -> gyro[0]);
        localizer.update();

        for (int i = 0; i < 10; i++) {
            encoders.advance(new double[]{-1, 1, -1, 1});
            clock.advance(0.02);
            localizer.update();
        }

        assertEquals(0.0, localizer.getPose().getHeading(), 1e-9);
    }

    @Test
    void headingSourceIsZeroedAtTheFirstUpdateSoTheFrameIsStartRelative() {
        // A gyro that happens to read 1.2 rad at init still defines heading zero.
        double[] gyro = {1.2};
        DriveEncoderLocalizer localizer =
                new DriveEncoderLocalizer(encoders, clock, () -> gyro[0]);
        localizer.update();

        assertEquals(0.0, localizer.getPose().getHeading(), 1e-9);

        gyro[0] = 1.2 + Math.PI / 2;
        encoders.advance(new double[]{0, 0, 0, 0});
        clock.advance(0.02);
        localizer.update();

        assertEquals(Math.PI / 2, localizer.getPose().getHeading(), 1e-9);
    }

    @Test
    void setPoseRebasesBothPositionAndHeadingOffset() {
        double[] gyro = {0.5};
        DriveEncoderLocalizer localizer =
                new DriveEncoderLocalizer(encoders, clock, () -> gyro[0]);
        localizer.update();

        localizer.setPose(new Pose2d(10, -5, Math.PI));
        assertEquals(10.0, localizer.getPose().getX(), 1e-9);
        assertEquals(Math.PI, localizer.getPose().getHeading(), 1e-9);

        // The gyro has not moved, so neither should the heading.
        clock.advance(0.02);
        localizer.update();
        assertEquals(Math.PI, Math.abs(localizer.getPose().getHeading()), 1e-9);
    }

    @Test
    void drivingInACircleReturnsRoughlyToTheStart() {
        double[] heading = {0.0};
        DriveEncoderLocalizer localizer =
                new DriveEncoderLocalizer(encoders, clock, () -> heading[0]);
        localizer.update();

        // Forward-and-turn in small steps, all the way around.
        int steps = 720;
        double forwardPerStep = 0.05;
        double turnPerStep = 2 * Math.PI / steps;
        for (int i = 0; i < steps; i++) {
            encoders.advance(new double[]{
                    forwardPerStep, forwardPerStep, forwardPerStep, forwardPerStep});
            heading[0] += turnPerStep;
            clock.advance(0.005);
            localizer.update();
        }

        assertTrue(localizer.getPose().position.norm() < 0.05,
                "closed loop should return to the origin, ended at "
                        + localizer.getPose());
    }
}
