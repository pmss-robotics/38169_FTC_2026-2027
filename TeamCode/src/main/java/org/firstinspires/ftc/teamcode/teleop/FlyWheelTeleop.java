package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

@TeleOp(name = "FulinnaTeleOp", group = "TeleOp")
public class FlyWheelTeleop extends CommandOpMode {
    private FlyWheelSubsystem outtakeMotor;

    @Override
    public void initialize() {
        outtakeMotor = new FlyWheelSubsystem(hardwareMap, telemetry);
        register(outtakeMotor);
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
