package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Team4Drive {

    // Hardware components are private and encapsulated
    private final DcMotor frontLeftDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backRightDrive;
    private final IMU imu;

    // constructor
    public Team4Drive(HardwareMap hardwareMap) {
        // Hardware Mapping
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        // Motor Directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU Setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        boolean imOK = imu.initialize(parameters);
    }

    /**
     * Robot drive parameters for drive method (DON'T EDITTTT PLS PLS PLS)
     *
     * @param rawX            X joystick input
     * @param rawY            Y joystick input
     * @param rawRx           Turning joystick input
     * @param inputCurve      Int for joystick multiplier curve (ex 1 is linear, 2 is quadratic, 3 cubic)
     * @param speedMultiplier Current speed scale (this is from speedModes enum
     */
    // method drive controls the mecanum drive
    public void drive(double rawX, double rawY, double rawRx, int inputCurve, double speedMultiplier) {
        // input curve
        double x = Math.copySign(Math.pow(Math.abs(rawX), inputCurve), rawX);
        double y = Math.copySign(Math.pow(Math.abs(rawY), inputCurve), rawY);
        double rx = Math.copySign(Math.pow(Math.abs(rawRx), inputCurve), rawRx);

        // get robot direction from IMU
        double botDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // movement relative to robot direction aka Field Centric Driving
        double rotX = x * Math.cos(-botDirection) - y * Math.sin(-botDirection);
        double rotY = x * Math.sin(-botDirection) + y * Math.cos(-botDirection);

        // Denominator scaling to equally put power in each motor
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        double flPower = (rotY + rotX + rx) / denominator;
        double blPower = (rotY - rotX + rx) / denominator;
        double frPower = (rotY - rotX - rx) / denominator;
        double brPower = (rotY + rotX - rx) / denominator;

        // put power in motors with speed multiplier
        frontLeftDrive.setPower(flPower * speedMultiplier);
        backLeftDrive.setPower(blPower * speedMultiplier);
        frontRightDrive.setPower(frPower * speedMultiplier);
        backRightDrive.setPower(brPower * speedMultiplier);
    }

    // utility methods

    // reset imu
    public void resetHeading() {
        imu.resetYaw();
    }
    //get direction
    public double getHeadingDegrees() {
        return Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
    @SuppressLint("DefaultLocale")
    public String getAnglesDebug() {
        YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
        return String.format("yaw %.1f  pitch %.1f  roll %.1f",
                a.getYaw(AngleUnit.DEGREES), a.getPitch(AngleUnit.DEGREES), a.getRoll(AngleUnit.DEGREES));
    }
}