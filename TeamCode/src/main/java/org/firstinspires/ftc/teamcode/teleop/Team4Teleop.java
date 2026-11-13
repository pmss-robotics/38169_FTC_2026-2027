package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Team4Drive;

@TeleOp(name = "Matthew Teleop", group = "Teleop")
public class Team4Teleop extends LinearOpMode {

    // get team4drive object
    private Team4Drive drive;

    // Speed modes dpad up to cycle
    private enum SpeedMode {
        slow("Slow", 0.25),
        medium("Normal", 0.5),
        fast("Fast", 1.0);

        final String label;
        final double multiplier;

        SpeedMode(String label, double multiplier) {
            this.label = label;
            this.multiplier = multiplier;
        }

        SpeedMode next() {
            SpeedMode[] all = values();
            return all[(ordinal() + 1) % all.length];
        }
    }

    private SpeedMode speedMode = SpeedMode.medium;
    private boolean lastKeyPressed = false;

    private int inputCurve;
    //main loop
    @Override
    public void runOpMode() {
        //initialize input curve
        inputCurve = 3;
        // initialize the subsystem and pass hardwareMap
        drive = new Team4Drive(hardwareMap);


        telemetry.addLine("Bot Ready rn");
        telemetry.addData("current direction in degrees: ", "%.1f", drive.getHeadingDegrees());telemetry.addData("IMU angles", drive.getAnglesDebug());
        telemetry.update();

        waitForStart();
        drive.resetHeading();

        while (opModeIsActive()) {
            // Reset IMU if back button is pressed
            if (gamepad1.back) {
                drive.resetHeading();
            }

            // Cycle speed mode with D-pad up
            boolean keyPressed = gamepad1.dpad_up;
            if (keyPressed && !lastKeyPressed) {
                speedMode = speedMode.next();
            }
            lastKeyPressed = keyPressed;

            // Pass gamepad inputs directly into the subsystem method
            drive.drive(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    inputCurve,
                    speedMode.multiplier
            );

            // Telemetry display
            telemetry.addData("current speed rn: ", "%s (%.0f%%)", speedMode.label, speedMode.multiplier * 100);
            telemetry.addData("current direction in degrees: ", "%.1f", drive.getHeadingDegrees());
            telemetry.addLine("Press d-pad up to cycle speed modes");
            telemetry.update();
        }
    }
}