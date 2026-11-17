package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;

/*
 * IMU ORIENTATION FINDER (debug tool, not for matches)
 *
 * Use it to find the LogoFacingDirection / UsbFacingDirection values that match how your
 * Control Hub is mounted on the robot.
 *
 * Controls (gamepad 1):
 *   D-pad up / down    : change the LOGO direction
 *   D-pad left / right : change the USB direction
 *   Y                  : zero the heading and clear the min/max readings
 *
 * How to test one combination:
 *   1. Put the robot flat on the floor and press Y.
 *   2. Turn the whole robot by hand about 90 degrees COUNTERCLOCKWISE (viewed from above).
 *   3. Look at the telemetry. The correct combination has:
 *        - YAW changing by about +90 (it must go UP for a counterclockwise turn)
 *        - PITCH and ROLL staying small (under about 20)
 *      The verdict line tells you when this is true.
 *   4. Copy the code line shown on screen into your Team4Drive IMU setup.
 *
 * Name check: the IMU must be called "imu" in Configure Robot.
 */
@TeleOp(name = "IMU Orientation Finder", group = "Test")
public class ImuOrientationFinder extends LinearOpMode {

    private IMU imu;

    // Smallest and largest reading seen for yaw / pitch / roll since the last reset.
    private final double[] minAngle = new double[3];
    private final double[] maxAngle = new double[3];

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");

        LogoFacingDirection[] logos = LogoFacingDirection.values();
        UsbFacingDirection[] usbs = UsbFacingDirection.values();

        // Start from the same values your robot code currently uses.
        int logoIdx = Arrays.asList(logos).indexOf(LogoFacingDirection.UP);
        int usbIdx = Arrays.asList(usbs).indexOf(UsbFacingDirection.FORWARD);

        boolean valid = applyOrientation(logos[logoIdx], usbs[usbIdx]);

        // Last loop's button states, so one press only counts once.
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false, lastY = false;

        telemetry.addLine("Ready. Press START, then use the D-pad to try combinations.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean changed = false;

            if (gamepad1.dpad_up && !lastUp) {
                logoIdx = (logoIdx + 1) % logos.length;
                changed = true;
            }
            if (gamepad1.dpad_down && !lastDown) {
                logoIdx = (logoIdx - 1 + logos.length) % logos.length;
                changed = true;
            }
            if (gamepad1.dpad_right && !lastRight) {
                usbIdx = (usbIdx + 1) % usbs.length;
                changed = true;
            }
            if (gamepad1.dpad_left && !lastLeft) {
                usbIdx = (usbIdx - 1 + usbs.length) % usbs.length;
                changed = true;
            }

            if (changed) {
                valid = applyOrientation(logos[logoIdx], usbs[usbIdx]);
            }
            if (gamepad1.y && !lastY && valid) {
                resetReadings();
            }

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastY = gamepad1.y;

            telemetry.addData("Logo direction", logos[logoIdx]);
            telemetry.addData("USB direction", usbs[usbIdx]);

            if (!valid) {
                // The SDK refuses combinations where logo and USB point along the same axis.
                telemetry.addLine("INVALID COMBINATION: logo and USB can't be on the same axis.");
                telemetry.addLine("(UP/DOWN, FORWARD/BACKWARD and LEFT/RIGHT are the three axes.)");
                telemetry.update();
                continue;
            }

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double[] now = {
                    angles.getYaw(AngleUnit.DEGREES),
                    angles.getPitch(AngleUnit.DEGREES),
                    angles.getRoll(AngleUnit.DEGREES)
            };
            for (int i = 0; i < 3; i++) {
                minAngle[i] = Math.min(minAngle[i], now[i]);
                maxAngle[i] = Math.max(maxAngle[i], now[i]);
            }

            telemetry.addData("Yaw now", "%.1f", now[0]);
            telemetry.addData("Pitch now", "%.1f", now[1]);
            telemetry.addData("Roll now", "%.1f", now[2]);
            telemetry.addData("Swing since reset (yaw / pitch / roll)", "%.0f / %.0f / %.0f",
                    maxAngle[0] - minAngle[0], maxAngle[1] - minAngle[1], maxAngle[2] - minAngle[2]);

            // A flat turn should swing yaw a lot and leave pitch and roll alone.
            double yawSwing = maxAngle[0] - minAngle[0];
            double otherSwing = Math.max(maxAngle[1] - minAngle[1], maxAngle[2] - minAngle[2]);
            if (yawSwing < 30) {
                telemetry.addData("Verdict", "Turn the robot flat by hand to test...");
            } else if (otherSwing < 20) {
                telemetry.addData("Verdict", "AXIS LOOKS RIGHT. Now check yaw went UP for a counterclockwise turn.");
            } else {
                telemetry.addData("Verdict", "WRONG: pitch/roll moved too. Try another combination.");
            }

            telemetry.addLine("Code line for this combination:");
            telemetry.addLine("new RevHubOrientationOnRobot(LogoFacingDirection." + logos[logoIdx]
                    + ", UsbFacingDirection." + usbs[usbIdx] + ")");
            telemetry.addLine("D-pad: up/down = logo, left/right = USB.  Y = reset.");
            telemetry.update();
        }
    }

    /** Re-initializes the IMU with a new orientation. Returns false if the SDK rejects the combination. */
    private boolean applyOrientation(LogoFacingDirection logo, UsbFacingDirection usb) {
        try {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
        } catch (RuntimeException e) {
            return false;
        }
        resetReadings();
        return true;
    }

    private void resetReadings() {
        imu.resetYaw();
        YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
        double[] now = {a.getYaw(AngleUnit.DEGREES), a.getPitch(AngleUnit.DEGREES), a.getRoll(AngleUnit.DEGREES)};
        for (int i = 0; i < 3; i++) {
            minAngle[i] = now[i];
            maxAngle[i] = now[i];
        }
    }
}