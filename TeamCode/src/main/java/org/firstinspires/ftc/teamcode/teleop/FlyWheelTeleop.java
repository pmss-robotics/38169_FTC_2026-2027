package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

@TeleOp(name = "FulinnaTeleOp", group = "TeleOp")
public class FlyWheelTeleop extends CommandOpMode {
    @Override
    public void initialize() {
        FlyWheelSubsystem outtakeMotor = new FlyWheelSubsystem(hardwareMap, telemetry);
        register(flywheel);
    }

    @Override
    public void run(){

        if (gamepad1.a) {
            outtakeMotor.setVelocityRPM(4500.0);
        } else if (gamepad1.b) {
            outtakeMotor.stop();
        }

        super.run();

    }

}