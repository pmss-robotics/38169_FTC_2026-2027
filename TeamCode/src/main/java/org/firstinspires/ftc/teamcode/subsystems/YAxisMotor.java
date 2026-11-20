package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class YAxisMotor extends SubsystemBase {
    private final Telemetry telemetry;
    private final DcMotorEx turretMotorY;
    // Ticks per degree of ramp tilt (Gear ratio * Encoder ticks per rev / 360.0)
    public static final double TICKS_PER_DEGREE = 5.0;

    // Safe mechanical min and max tilt angles in degrees
    public static final double MIN_ANGLE = 0.0;
    public static final double MAX_ANGLE = 60.0;
    private double targetAngle = 0.0;

    public YAxisMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        turretMotorY = hardwareMap.get(DcMotorEx.class, "turretMotorY");
        turretMotorY.setDirection(DcMotorEx.Direction.FORWARD);
        turretMotorY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reset encoder at boot (assumes ramp starts at MIN_ANGLE / resting position)
        turretMotorY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorY.setTargetPosition(0);
        turretMotorY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotorY.setPower(0.5);
    }
    public void setTargetAngle(double degrees) {
        targetAngle = Math.max(MIN_ANGLE, Math.min(degrees, MAX_ANGLE));
        int targetTicks = (int) (targetAngle * TICKS_PER_DEGREE);
        turretMotorY.setTargetPosition(targetTicks + turretMotorY.getCurrentPosition());
        turretMotorY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotorY.setPower(0.5);
    }
    public void adjustAngle(double deltaDegrees) {
        setTargetAngle(targetAngle + deltaDegrees);
    }
    public double getTargetAngle() {
        return targetAngle;
    }
    public double getCurrentAngle() {
        return turretMotorY.getCurrentPosition() / TICKS_PER_DEGREE;
    }
    @Override
    public void periodic() {
        telemetry.addData("Ramp Target Angle (in degrees)", targetAngle);
        telemetry.addData("Ramp Current Angle (in degrees)", getCurrentAngle());
    }
}
