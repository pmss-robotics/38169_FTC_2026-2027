package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Team4Drive;

@TeleOp(name = "Matthew Teleop", group = "Teleop")
public class Team4Teleop extends LinearOpMode {

    // teleop match length in seconds
    private static final double teleopLength = 120.0;

    // get team4drive object and timer
    private Team4Drive drive;
    private final ElapsedTime teleopTimer = new ElapsedTime();
    //input curve
    private int inputCurve;

    // main loop
    @Override
    public void runOpMode() {
        // initialize input curve
        inputCurve = 3;

        // initialize the subsystem and pass hardwareMap
        drive = new Team4Drive(hardwareMap);

        telemetry.addLine("Bot ready rn:");
        telemetry.addData("Current direction (deg)", "%.1f", drive.getHeadingDegrees());
        telemetry.addData("IMU angles", drive.getAnglesDebug());
        telemetry.update();

        waitForStart();
        teleopTimer.reset();
        drive.resetHeading();

        while (opModeIsActive()) {
            // reset IMU if back button is pressed
            if (gamepad1.back) {
                drive.resetHeading();
            }

            // pass gamepad inputs directly into the subsystem method
            drive.drive(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    inputCurve
            );

            // calculate remaining teleop time
            double timeLeft = Math.max(0.0, teleopLength - teleopTimer.seconds());

            // telemetry display
            telemetry.addData("Time left", "%.1f", timeLeft);
            if (timeLeft <= 20.0) {
                telemetry.addLine("ENDGAME - Return to LOADING ZONE");
            } else if (timeLeft <= 60.0) {
                telemetry.addLine("FLOWERS UNLOCKED (NECTAR allowed)");
            }
            telemetry.addData("Current direction (deg)", "%.1f", drive.getHeadingDegrees());
            telemetry.update();
        }
    }
}