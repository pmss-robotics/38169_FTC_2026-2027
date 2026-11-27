package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * Points: +3 for leave (robot no longer touching the perimeter wall)
 *         +5 for park (robot at least somewhat inside the loading zone)
 *         Important: Points are assessed by the end of Auto
 *
 * Parameters:
 *         - parkX
 *         - parkY
 *         - parkHeading
 *
 * The parkX and parkY control the park destination coordinates (robot relative)
 * and the parkHeading controls the direction the robots faces
 * Coordinates are
 */
@Autonomous(name = "Auto Leave + Park", group = "Pedro")
public class AutoLeavePark extends LinearOpMode {
    private static final double parkX = 20;      // length of field is 144 in
    private static final double parkY = 0;       // length of field is 144 in
    private static final double parkHeading = 0; //direction

    private static final double autoLength = 30.0; // seconds

    private Follower follower;
    private final ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        Pose start = new Pose(0, 0, 0);
        follower.setStartingPose(start);

        telemetry.addLine("Place robot against the wall in the designated starting area");
        telemetry.addData("Park spot", "x=%.1f y=%.1f", parkX, parkY);
        telemetry.update();

        waitForStart();
        autoTimer.reset();

        Pose parkPose = new Pose(parkX, parkY, parkHeading);

        // Leave the wall and park in the zone
        if (opModeIsActive()) {
            PathChain toPark = follower.pathBuilder()
                    .addPath(new BezierLine(start, parkPose))
                    .setConstantHeadingInterpolation(parkHeading)
                    .build();
            followUntilDone(toPark, autoLength - autoTimer.seconds() - 1.0);
        }

        // Hold the spot till the end of Auto
        follower.holdPoint(parkPose);
        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("Time", "%.1f", autoTimer.seconds());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }

    //path follower function
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