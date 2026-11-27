package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Transfer mechanism: one goBILDA 1150 RPM Yellow Jacket motor that spins clockwise
 * (looking at the motor shaft) to feed game pieces.
 */
public class TransferSubsystem {

    // Use 0.3 for the first test, then raise it
    private static final double TRANSFER_POWER = 1.0;

    private DcMotorEx motor;

    public void init(HardwareMap hwMap) {
        // MOTOR NAME CONFIG
        motor = hwMap.get(DcMotorEx.class, "transfer");

        // With FORWARD, positive power spins the shaft counterclockwise, so REVERSE gives clockwise
        //TUNE w/ FORWARD or BACKWARD
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0); // start stopped
    }

    //spin clockwise
    public void run() {
        motor.setPower(TRANSFER_POWER);
    }

    //spin counterclockwise
    public void reverse() {
        motor.setPower(-TRANSFER_POWER);
    }

    public void stop() {
        motor.setPower(0);
    }

    public double getPower() {
        return motor.getPower();
    }
}
