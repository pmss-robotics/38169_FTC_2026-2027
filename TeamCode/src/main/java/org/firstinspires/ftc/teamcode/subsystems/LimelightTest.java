package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Limelight version of VisionTest.
 * Set LIMELIGHT_NAME to match the Control Hub config, and make sure
 * PIPELINE has AprilTag detection enabled in the Limelight web UI.
 */
@TeleOp(name = "Limelight Test")
public class LimelightTest extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight";
    private static final int PIPELINE = 0;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing Limelight...");
        telemetry.update();

        LimelightSubsystem limelight =
                new LimelightSubsystem(hardwareMap, LIMELIGHT_NAME, PIPELINE);
        limelight.start();

        telemetry.addLine("Limelight Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            limelight.update();

            telemetry.addData("Connected", limelight.isConnected());
            telemetry.addData("Running", limelight.isRunning());

            LLStatus status = limelight.getStatus();
            if (status != null) {
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s",
                        status.getPipelineIndex(), status.getPipelineType());
            }

            List<LLResultTypes.FiducialResult> tags = limelight.getTags();

            telemetry.addData("Detections", tags.size());

            if (!limelight.hasValidResult()) {
                telemetry.addLine("No valid Limelight data");
            } else if (tags.isEmpty()) {
                telemetry.addLine("No AprilTags visible");
            }

            for (LLResultTypes.FiducialResult tag : tags) {

                telemetry.addLine("--- TAG ---");
                telemetry.addData("ID", tag.getFiducialId());
                telemetry.addData("Family", tag.getFamily());
                telemetry.addData("Tx (deg)", "%.2f", tag.getTargetXDegrees());
                telemetry.addData("Ty (deg)", "%.2f", tag.getTargetYDegrees());

                // Tag position relative to the camera
                Pose3D pose = tag.getTargetPoseCameraSpace();
                if (pose != null) {
                    telemetry.addData("X (in)", "%.2f",
                            pose.getPosition().toUnit(DistanceUnit.INCH).x);
                    telemetry.addData("Y (in)", "%.2f",
                            pose.getPosition().toUnit(DistanceUnit.INCH).y);
                    telemetry.addData("Z (in)", "%.2f",
                            pose.getPosition().toUnit(DistanceUnit.INCH).z);
                }
            }

            telemetry.update();
            sleep(20);
        }

        limelight.stop();
    }
}
