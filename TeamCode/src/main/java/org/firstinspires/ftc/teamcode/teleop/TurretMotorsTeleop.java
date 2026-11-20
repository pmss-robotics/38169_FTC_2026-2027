package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.YAxisMotor;
import org.firstinspires.ftc.teamcode.subsystems.XAxisMotor;

@TeleOp(name = "TurretMotorsTeleOp", group = "TeleOp")
public class TurretMotorsTeleop extends CommandOpMode {
    private XAxisMotor xAxisMotor;
    private YAxisMotor yAxisMotor;

    @Override
    public void initialize() {
        xAxisMotor = new XAxisMotor(hardwareMap, telemetry);
        yAxisMotor = new YAxisMotor(hardwareMap, telemetry);

        register(xAxisMotor, yAxisMotor);
    }
    @Override
    public void run() {
        if (gamepad1.right_stick_x < 0.0) {
            xAxisMotor.setTargetAngle(1.5);
        } else if (gamepad1.right_stick_x > 0.0) {
            xAxisMotor.setTargetAngle(1.5);
        }
        if (gamepad1.right_trigger <= 0.5) {
            yAxisMotor.setTargetAngle(1.0);
        } else if (gamepad1.right_trigger <= 1.0) {
            yAxisMotor.setTargetAngle(1.5);

        }
        super.run();
    }
}