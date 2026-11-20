package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Pedro v2 Full Debugger", group = "Pedro")
public class PedroV2Debugger extends OpMode {

    private Follower follower;
    private PathChain forward24;
    private PathChain returnOrigin;

    private DcMotorEx lf, rf, lr, rr;

    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;

    private enum TestState {
        TRACKING_ONLY,
        FOLLOWING_FORWARD,
        FOLLOWING_RETURN,
        MOTOR_ISOLATION
    }

    private TestState currentState = TestState.TRACKING_ONLY;
    private int activeWheel = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Build paths once during init
        forward24 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

        returnOrigin = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24, 0, 0), new Pose(0, 0, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

        // Update these names to match your Robot Controller config
        try {
            lf = hardwareMap.get(DcMotorEx.class, "leftFront");
            rf = hardwareMap.get(DcMotorEx.class, "rightFront");
            lr = hardwareMap.get(DcMotorEx.class, "leftRear");
            rr = hardwareMap.get(DcMotorEx.class, "rightRear");
        } catch (Exception e) {
            telemetry.addLine("Warning: Motor config names did not match; D-Pad wheel test disabled.");
        }

        telemetry.addLine("=== PEDRO V2 DEBUGGER ===");
        telemetry.addLine("A: Drive forward 24 in");
        telemetry.addLine("X: Return to (0, 0)");
        telemetry.addLine("Y: Reset pose to (0, 0, 0) while stationary");
        telemetry.addLine("D-Pad Up: Cycle single-wheel spin test (25% power)");
        telemetry.addLine("B: Emergency Stop");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Keep follower updating unless testing raw motor directions
        if (currentState != TestState.MOTOR_ISOLATION) {
            follower.update();
        }

        // A: Run forward 24" path
        if (gamepad1.a && !lastA) {
            currentState = TestState.FOLLOWING_FORWARD;
            follower.followPath(forward24);
        }
        lastA = gamepad1.a;

        // X: Return to origin
        if (gamepad1.x && !lastX) {
            currentState = TestState.FOLLOWING_RETURN;
            follower.followPath(returnOrigin);
        }
        lastX = gamepad1.x;

        // Y: Safe pose reset while stationary (replaces setStartingPose in loop)
        if (gamepad1.y && !lastY) {
            follower.breakFollowing();
            follower.setPose(new Pose(0, 0, 0));
            currentState = TestState.TRACKING_ONLY;
        }
        lastY = gamepad1.y;

        // D-Pad Up: Cycle individual wheels to verify spin & encoder direction
        if (gamepad1.dpad_up && !lastDpadUp && lf != null) {
            follower.breakFollowing();
            currentState = TestState.MOTOR_ISOLATION;
            activeWheel = (activeWheel + 1) % 5; // 1-4 = wheels, 0 = off
            testSingleMotor(activeWheel);
        }
        lastDpadUp = gamepad1.dpad_up;

        // B: Emergency stop
        if (gamepad1.b) {
            follower.breakFollowing();
            stopAllMotors();
            currentState = TestState.TRACKING_ONLY;
        }

        // Revert state label automatically when path finishes
        if ((currentState == TestState.FOLLOWING_FORWARD || currentState == TestState.FOLLOWING_RETURN) && !follower.isBusy()) {
            currentState = TestState.TRACKING_ONLY;
        }

        // --- LIVE DIAGNOSTIC TELEMETRY ---
        Pose pose = follower.getPose();
        telemetry.addData("State", currentState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addLine("--- POSE & LOCALIZER ---");
        telemetry.addData("X (in)", "%.2f", pose.getX());
        telemetry.addData("Y (in)", "%.2f", pose.getY());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.getHeading()));

        if (lf != null) {
            telemetry.addLine("--- MOTOR ENCODERS ---");
            telemetry.addData("LF | RF", "%d | %d", lf.getCurrentPosition(), rf.getCurrentPosition());
            telemetry.addData("LR | RR", "%d | %d", lr.getCurrentPosition(), rr.getCurrentPosition());
        }

        telemetry.update();
    }

    private void testSingleMotor(int wheelIndex) {
        stopAllMotors();
        double testPower = 0.25;
        switch (wheelIndex) {
            case 1:
                lf.setPower(testPower);
                telemetry.addLine("Testing: LEFT FRONT (Should spin forward)");
                break;
            case 2:
                rf.setPower(testPower);
                telemetry.addLine("Testing: RIGHT FRONT (Should spin forward)");
                break;
            case 3:
                lr.setPower(testPower);
                telemetry.addLine("Testing: LEFT REAR (Should spin forward)");
                break;
            case 4:
                rr.setPower(testPower);
                telemetry.addLine("Testing: RIGHT REAR (Should spin forward)");
                break;
            default:
                currentState = TestState.TRACKING_ONLY;
                break;
        }
    }

    private void stopAllMotors() {
        if (lf != null) {
            lf.setPower(0);
            rf.setPower(0);
            lr.setPower(0);
            rr.setPower(0);
        }
    }
}