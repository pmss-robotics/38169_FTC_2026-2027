package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BlockerMatthew;

/*
 * BLOCKER TEST (gamepad 1)
 *   A          : deploy
 *   B          : retract
 *   D-pad up   : nudge toward deployed
 *   D-pad down : nudge toward retracted
 *   X          : set the current position as 0 (do it with the blocker fully retracted)
 *   Y          : emergency stop (cuts power)
 *   Left bumper: toggle between LOW power (testing) and normal power
 */
@TeleOp(name = "Blocker Test", group = "Test")
public class  BlockerTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        BlockerMatthew blocker = new BlockerMatthew(hardwareMap);
        blocker.setPower(BlockerMatthew.LOW_POWER);   // start slow
        boolean lowPower = true;

        // Last loop's button states, so one press only counts once.
        boolean lastA = false, lastB = false, lastUp = false, lastDown = false;
        boolean lastX = false, lastY = false, lastBumper = false;

        telemetry.addLine("Retract the blocker by hand, then press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !lastA) blocker.deploy();
            if (gamepad1.b && !lastB) blocker.retract();
            if (gamepad1.dpad_up && !lastUp) blocker.nudge(1);
            if (gamepad1.dpad_down && !lastDown) blocker.nudge(-1);
            if (gamepad1.x && !lastX) blocker.resetEncoders();
            if (gamepad1.y && !lastY) blocker.stop();
            if (gamepad1.left_bumper && !lastBumper) {
                lowPower = !lowPower;
                blocker.setPower(lowPower ? BlockerMatthew.LOW_POWER : BlockerMatthew.POWER);
            }

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastX = gamepad1.x;
            lastY = gamepad1.y;
            lastBumper = gamepad1.left_bumper;

            telemetry.addData("Power mode", lowPower ? "LOW (testing)" : "NORMAL");
            telemetry.addData("Target ticks", blocker.getTarget());
            telemetry.addData("Left ticks", blocker.getLeftPosition());
            telemetry.addData("Right ticks", blocker.getRightPosition());
            telemetry.addData("At target", blocker.atTarget());
            telemetry.addData("Jammed?", blocker.isJammed());
            telemetry.update();
        }
    }
}