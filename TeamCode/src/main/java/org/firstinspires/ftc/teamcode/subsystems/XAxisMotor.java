package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//X-Axis motor for turret

public class XAxisMotor extends SubsystemBase{
    private final Telemetry telemetry;
    private final DcMotorEx turretMotorX;
    // Ticks per degree (Gear ratio * Encoder ticks per rev / 360.0)
    public static final double TICKS_PER_DEGREE = 5.0;

    private int targetTicks = 0;

    public XAxisMotor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motor
        turretMotorX = hardwareMap.get(DcMotorEx.class, "turretMotorX");
        turretMotorX.setDirection(DcMotorEx.Direction.FORWARD);

        // Use FLOAT for the flywheel to coast to a gradual stop
        // Use BRAKE to stop quickly
        turretMotorX.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reset encoder to 0 at start
        turretMotorX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorX.setTargetPosition(0);
        turretMotorX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotorX.setPower(0.5);
    }
    public void setTargetAngle(double degrees) {
        targetTicks = (int) (degrees * TICKS_PER_DEGREE);
        turretMotorX.setTargetPosition(targetTicks + turretMotorX.getCurrentPosition());
        turretMotorX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotorX.setPower(0.5);
    }

    // Soft limits in encoder ticks from the starting (centered) position, so the turret can't
    // spin into its cables. TODO: placeholders, measure the real safe range and set these.
    public static final int MIN_TICKS = -500;
    public static final int MAX_TICKS = 500;

    public void movement(double speed, boolean direction) {
        double power = Math.min(1.0, Math.abs(speed));

        turretMotorX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Refuse to drive further past a soft limit (driving back toward center is allowed)
        int pos = turretMotorX.getCurrentPosition();
        if ((direction && pos >= MAX_TICKS) || (!direction && pos <= MIN_TICKS)) {
            power = 0;
        }

        if (direction) {
            turretMotorX.setPower(power);
        } else {
            turretMotorX.setPower(-power);
        }
    }

    /** Stop turning (BRAKE holds it in place). */
    public void stop() {
        turretMotorX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotorX.setPower(0);
    }
    public double getCurrentAngle() {
        return turretMotorX.getCurrentPosition();
    }
    @Override

    public void periodic() {
        telemetry.addData("Turret Target Angle", targetTicks);
        telemetry.addData("Turret Current Angle", getCurrentAngle());
    }

}
