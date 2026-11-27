package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@TeleOp(name = "Transfer Test")
public class TransferTestTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        TransferSubsystem transfer = new TransferSubsystem();
        transfer.init(hardwareMap);

        telemetry.addLine("Hold A = run (clockwise), hold B = reverse");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                transfer.run();
            } else if (gamepad1.b) {
                transfer.reverse();
            } else {
                transfer.stop();
            }

            telemetry.addData("Transfer power", transfer.getPower());
            telemetry.update();
        }
    }
}
