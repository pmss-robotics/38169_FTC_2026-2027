package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * BIOBUZZ AUTO, LAYER 1 (leave + park) with a spot for LAYER 2 (shoot preloads).
 * Pedro 2.1.2 version (same calls as your PedroQuickTest).
 *
 * Points: LEAVE 3 (robot no longer touching the perimeter wall) + AUTO PARK 5 (robot at least
 * partly inside your LOADING ZONE at the END of AUTO). Both are checked when AUTO ends.
 *
 * COORDINATES: relative frame, so you don't need field coordinates:
 *   - The robot's start spot is (0, 0) facing heading 0.
 *   - +x is the way the robot faces at the start, +y is to its LEFT. Heading is in RADIANS.
 * Put the robot on the exact same start spot every match (touching the wall, rule G304).
 *
 * MEASURE the PARK numbers with a tape measure (inches, robot CENTER from the start spot to
 * the end spot where part of the robot is inside the LOADING ZONE). The numbers below are
 * placeholders. Don't run them as-is.
 */
@Autonomous(name = "Auto Leave + Park", group = "Pedro")
public class AutoLeavePark extends LinearOpMode {

    // ---- MEASURE AND FILL IN (inches, radians) ----
    private static final double SHOOT_X = 0;
    private static final double SHOOT_Y = 0;
    private static final double SHOOT_HEADING = 0;
    private static final double PARK_X = 20;      // placeholder
    private static final double PARK_Y = 0;       // placeholder
    private static final double PARK_HEADING = 0;

    private static final boolean DO_SHOOT = false; // turn on only after layer 2 is tested
    private static final double AUTO_LENGTH = 30.0; // seconds

    private Follower follower;
    private final ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        Pose start = new Pose(0, 0, 0);
        follower.setStartingPose(start);

        telemetry.addLine("Start with the robot touching the wall, on its start spot.");
        telemetry.addData("Park spot", "x=%.1f y=%.1f", PARK_X, PARK_Y);
        telemetry.update();

        waitForStart();
        autoTimer.reset();

        Pose parkPose = new Pose(PARK_X, PARK_Y, PARK_HEADING);
        Pose shootPose = new Pose(SHOOT_X, SHOOT_Y, SHOOT_HEADING);
        Pose parkFrom = start;

        // ----- LAYER 2 (optional): drive to the shoot spot and fire the 4 preloads -----
        if (DO_SHOOT && opModeIsActive()) {
            PathChain toShoot = follower.pathBuilder()
                    .addPath(new BezierLine(start, shootPose))
                    .setConstantHeadingInterpolation(SHOOT_HEADING)
                    .build();
            followUntilDone(toShoot, 8.0);

            // TODO: shoot 4 POLLEN here with your shooter code.
            // Spin up, wait until at speed, feed one POLLEN at a time, then stop the flywheel.
            // Keep a time limit so a jam can't use up the park time.
            parkFrom = shootPose;
        }

        // ----- LAYER 1: leave the wall and park in the LOADING ZONE -----
        if (opModeIsActive()) {
            PathChain toPark = follower.pathBuilder()
                    .addPath(new BezierLine(parkFrom, parkPose))
                    .setConstantHeadingInterpolation(PARK_HEADING)
                    .build();
            followUntilDone(toPark, AUTO_LENGTH - autoTimer.seconds() - 1.0);
        }

        // Hold the end spot until AUTO ends (the robot must be at rest and in the zone).
        follower.holdPoint(parkPose);
        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("Time", "%.1f", autoTimer.seconds());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }

    /** Follows a path until it finishes or the time limit passes. */
    private void followUntilDone(PathChain path, double timeLimitSeconds) {
        double startTime = autoTimer.seconds();
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()
                && autoTimer.seconds() - startTime < timeLimitSeconds) {
            follower.update();
            telemetry.addData("Time", "%.1f", autoTimer.seconds());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }
}