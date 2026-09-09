package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.verniteyaku.pathing.control.FollowerConstants;
import com.verniteyaku.pathing.follower.PathFollower;
import com.verniteyaku.pathing.geometry.Pose2d;
import com.verniteyaku.pathing.ftc.ImuHeadingSource;
import com.verniteyaku.pathing.ftc.MecanumDrivetrain;
import com.verniteyaku.pathing.localization.FusedLocalizer;
import com.verniteyaku.pathing.paths.BezierCurve;
import com.verniteyaku.pathing.paths.BezierLine;
import com.verniteyaku.pathing.paths.PathBuilder;
import com.verniteyaku.pathing.paths.PathChain;
import com.verniteyaku.pathing.paths.Point;
import com.verniteyaku.pathing.units.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A complete worked example: build a path chain, follow it, report progress.
 *
 * <p>The numbers below describe a generic goBILDA-style mecanum robot. Replace
 * them with your own -- especially the motor names, which must match your robot
 * configuration exactly.
 *
 * <p>Remember the coordinate frame: coordinates are absolute field positions,
 * origin at the centre of the field. So {@code (-36, -36)} is a fixed spot, and
 * the library has to be told where the robot actually starts -- that is the
 * {@code startPose} below. See {@code FieldCoordinates}.
 */
@Autonomous(name = "VerniteYaku Example Auto", group = "VerniteYaku")
public class ExampleAutoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware -------------------------------------------------------

        MecanumDrivetrain drivetrain = MecanumDrivetrain.builder(DistanceUnit.INCH)
                .motors(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight")
                .reverse(true, false, true, false)
                .trackWidth(15.0)
                .wheelBase(13.0)
                .wheelRadius(1.89)          // 96 mm goBILDA mecanum
                .ticksPerRevolution(537.7)  // 312 RPM Yellow Jacket
                .maxMotorRpm(312)
                .build();
        drivetrain.resetEncoders();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Initialise the IMU with your hub's actual orientation before this
        // point if you have not already; the library takes it as it finds it.
        ImuHeadingSource heading = new ImuHeadingSource(imu);

        // Where the robot is actually placed, in field coordinates. Measure this
        // once against the tiles; get it wrong and the whole auto is offset by
        // the same amount.
        Pose2d startPose = new Pose2d(-60, -36, Math.toRadians(0));

        // Encoders predict, the IMU corrects. Swap in DriveEncoderLocalizer if
        // you want plain dead reckoning with no filtering.
        FusedLocalizer localizer = FusedLocalizer.builder(
                        drivetrain, com.verniteyaku.pathing.control.Clock.system())
                .startPose(startPose)
                .headingSource(heading)
                .build();

        // --- Tuning ---------------------------------------------------------

        FollowerConstants constants = FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(40)
                .maxAcceleration(50)
                .maxDeceleration(60)
                // Start from 1 / (top wheel speed in inches per second) and tune
                // from there. Phase 3's auto-tuner will fit these for you.
                .kV(0.017)
                .kS(0.05)
                .kA(0.0)
                .translationalPID(0.15, 0.0, 0.01)
                .headingPID(1.5, 0.0, 0.05)
                .positionTolerance(1.0)
                .headingTolerance(Math.toRadians(3))
                // Reactive blending: correct up to 2.5x as hard once the robot
                // is 4 inches or more off the path. Set reactiveAuthority(1.0)
                // to turn this off and get a plain profile follower back.
                .reactiveErrorScale(4.0)
                .reactiveAuthority(2.5)
                .build();

        PathFollower follower = new PathFollower(drivetrain, localizer, constants);

        // --- The path -------------------------------------------------------

        // Absolute field coordinates. The first point is where the robot starts,
        // so it does not have to drive onto the path before following it.
        PathChain chain = new PathBuilder()
                // Straight out two feet, turning to face field +Y as we go.
                .addPath(new BezierLine(new Point(-60, -36), new Point(-36, -36)))
                .setLinearHeadingInterpolation(0, Math.toRadians(90))
                // Then curve away, holding that heading.
                .addPath(new BezierCurve(
                        new Point(-36, -36), new Point(-24, -24), new Point(-24, -6)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Path", "%.1f inches over %d segments",
                chain.length(), chain.size());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // --- Follow ---------------------------------------------------------

        follower.followPath(chain);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Progress", "%.1f / %.1f in",
                    follower.getArcLengthTravelled(), chain.length());
            telemetry.addData("Error", "%.2f in, %.1f deg",
                    follower.getPositionError(),
                    AngleUnit.DEGREES.fromRadians(follower.getHeadingError()));
            // Worth watching while tuning the blend: 1.0 means "tracking fine",
            // rising toward reactiveAuthority means "fighting to get back".
            telemetry.addData("Authority", "%.2f (confidence %.2f)",
                    follower.getCorrectionAuthority(), follower.getPoseConfidence());
            telemetry.update();
        }

        // The follower stops the motors itself, whether it arrived or timed out.
        telemetry.addData("Status", follower.isAtTarget() ? "Arrived" : "Timed out short");
        telemetry.addData("Final pose", follower.getPose());
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
