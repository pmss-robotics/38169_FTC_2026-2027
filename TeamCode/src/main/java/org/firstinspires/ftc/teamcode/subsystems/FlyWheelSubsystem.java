package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlyWheelSubsystem extends SubsystemBase{
    private final Telemetry telemetry;
    private final DcMotorEx outtakeMotor;
    private static final double kP = 25.0, kI = 0.0, kD = 0.2;
    private static final double kF = 12.0;
    public static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000.0;
    private double targetRPM = 0.0;

    public FlyWheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motor
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Use FLOAT for the flywheel to coast to a gradual stop
        // Use BRAKE to stop quickly
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Use encoder feedback to regulate motor speed
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Use PIDF for manual, precise adjustments
        outtakeMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }
    public void setVelocityRPM(double rpm) {
        targetRPM = Math.max(0, Math.min(rpm, MAX_RPM));
        double ticksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;
        outtakeMotor.setVelocity(ticksPerSec);
    }
    public void stop() {
        setVelocityRPM(0.0);
    }
    @Override

    public void periodic() {

        if (outtakeMotor == null) {
            telemetry.addLine("ERROR: Flywheel motor is NULL!");
            telemetry.update();
            return;
        }

        double actualRPM = (outtakeMotor.getVelocity() * 60.0) / TICKS_PER_REV;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", actualRPM);
    }

}