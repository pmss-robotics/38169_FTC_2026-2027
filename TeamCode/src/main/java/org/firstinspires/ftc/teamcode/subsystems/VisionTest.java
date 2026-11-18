package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

@TeleOp(name = "Vision Test")
public class VisionTest extends LinearOpMode {

    private VisionPortal portal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Initialize webcam
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Initializing Vision System...");
        telemetry.update();

        // Wait for camera to initialize
        while (!isStopRequested() &&
                portal.getCameraState() !=
                        VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        telemetry.addLine("Vision System Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections =
                    aprilTag.getDetections();

            telemetry.addData(
                    "Camera",
                    portal.getCameraState()
            );

            telemetry.addData(
                    "Detections",
                    detections.size()
            );

            if (detections.isEmpty()) {
                telemetry.addLine("No AprilTags visible");
            }

            for (AprilTagDetection detection : detections) {

                // Individual AprilTag
                if (detection instanceof AprilTagSingleDetection) {

                    AprilTagSingleDetection tag =
                            (AprilTagSingleDetection) detection;

                    telemetry.addLine("--- SINGLE TAG ---");

                    telemetry.addData("ID", tag.id);

                    if (tag.metadata != null) {
                        telemetry.addData(
                                "Name",
                                tag.metadata.name
                        );
                    }

                }

                // BioBuzz AprilTag Cluster
                else if (detection instanceof AprilTagClusterDetection) {

                    AprilTagClusterDetection cluster =
                            (AprilTagClusterDetection) detection;

                    telemetry.addLine("--- BIOBUZZ CLUSTER ---");

                    telemetry.addData(
                            "Cluster",
                            cluster.metadata.name
                    );

                    telemetry.addData(
                            "Tags Found",
                            cluster.percentClusterFound + "%"
                    );
                }

                // Position relative to camera
                if (detection.ftcPose != null) {

                    telemetry.addData(
                            "Range (in)",
                            "%.2f",
                            detection.ftcPose.range
                    );

                    telemetry.addData(
                            "X (in)",
                            "%.2f",
                            detection.ftcPose.x
                    );

                    telemetry.addData(
                            "Y (in)",
                            "%.2f",
                            detection.ftcPose.y
                    );

                    telemetry.addData(
                            "Z (in)",
                            "%.2f",
                            detection.ftcPose.z
                    );

                    telemetry.addData(
                            "Bearing",
                            "%.2f degrees",
                            detection.ftcPose.bearing
                    );

                    telemetry.addData(
                            "Yaw",
                            "%.2f degrees",
                            detection.ftcPose.yaw
                    );

                    telemetry.addData(
                            "Roll",
                            "%.2f degrees",
                            detection.ftcPose.roll
                    );
                }
            }

            telemetry.update();
            sleep(20);
        }

        portal.close();
    }
}