package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Team4Drive;

@TeleOp(name = "Matthew Teleop", group = "Teleop")
public class Team4Teleop extends LinearOpMode {

    // get team4drive object
    private Team4Drive drive;
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
        telemetry.update();

        waitForStart();
        drive.resetHeading();

        while (opModeIsActive()) {
            // Reset IMU if back button is pressed
            if (gamepad1.back) {
                drive.resetHeading();

                // Pass gamepad inputs directly into the subsystem method
                drive.drive(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        inputCurve
                );

                // Telemetry display
                telemetry.addData("current direction in degrees: ", "%.1f", drive.getHeadingDegrees());
                telemetry.addLine("Press d-pad up to cycle speed modes");
                telemetry.update();
            }
        }
    }
}