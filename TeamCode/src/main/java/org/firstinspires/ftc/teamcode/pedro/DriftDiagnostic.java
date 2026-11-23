package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Drift Diagnostic: tells apart a MECHANICAL drift from a LOCALIZER (odometry) problem.
 *
 * Pedro is NOT used to drive here. The robot runs OPEN LOOP: every wheel gets a fixed power,
 * exactly like teleop, with no feedback. Pedro's localizer only WATCHES. So:
 *   - If the robot turns/drifts on its own with equal power -> that is mechanical (or floor).
 *   - If the Pinpoint reports the robot moving a DIFFERENT way than it visibly moved
 *     -> that is a localizer problem. Pedro then "corrects" the wrong way and runs away.
 *
 * Four short runs (~2 ft each). Before each one, place the robot with clear space and press A.
 *   1. FORWARD       2. BACKWARD       3. STRAFE RIGHT       4. STRAFE LEFT
 * WATCH THE ROBOT during each run and remember which way it really went.
 * B = skip a run / stop.  After all runs, the summary stays on screen.
 */
@TeleOp(name = "Drift Diagnostic", group = "Pedro")
public class DriftDiagnostic extends LinearOpMode {

    public static double POWER = 0.35;          // same power on every wheel
    public static double DRIVE_SECONDS = 1.2;   // how long each run lasts
    public static double MAX_DISTANCE_IN = 36;  // safety stop
    private static final double STOPPED_TICKS_PER_SEC = 30;

    private enum Run {
        FORWARD("FORWARD", 1, 0), BACKWARD("BACKWARD", -1, 0),
        STRAFE_RIGHT("STRAFE RIGHT", 0, 1), STRAFE_LEFT("STRAFE LEFT", 0, -1);
        final String label; final int fwd; final int right;
        Run(String label, int fwd, int right) { this.label = label; this.fwd = fwd; this.right = right; }
    }

    private Follower follower;
    private IMU hubImu;
    private DcMotorEx fl, bl, fr, br;
    private DcMotorEx[] motors;
    private static final String[] NAMES = {"fl", "bl", "fr", "br"};
    private double hubTotal = 0, hubPrev = 0;
    private final List<String> summary = new ArrayList<>();

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap); // used ONLY to read the Pinpoint pose
        follower.setStartingPose(new Pose(0, 0, 0));

        fl = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftFrontMotorName);
        bl = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftRearMotorName);
        fr = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightFrontMotorName);
        br = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightRearMotorName);
        fl.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        bl.setDirection(Constants.driveConstants.leftRearMotorDirection);
        fr.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        br.setDirection(Constants.driveConstants.rightRearMotorDirection);
        motors = new DcMotorEx[]{fl, bl, fr, br};
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // same as teleop
        }

        try {
            hubImu = hardwareMap.get(IMU.class, "imu");
            hubImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            hubImu.resetYaw();
        } catch (Exception e) {
            hubImu = null;
        }

        telemetry.addLine("DRIFT DIAGNOSTIC - robot on the FLOOR, ~3 ft clear all around.");
        telemetry.addLine("Each run: fixed equal power, no Pedro control.");
        telemetry.addLine("WATCH which way the robot really moves.");
        telemetry.update();
        waitForStart();

        for (Run run : Run.values()) {
            if (!opModeIsActive()) break;
            if (!waitForA(run)) {
                summary.add(run.label + ": skipped");
                continue;
            }
            doRun(run);
        }

        stopAll();
        while (opModeIsActive()) {
            telemetry.addLine("=== SUMMARY (scroll) ===");
            for (String s : summary) telemetry.addLine(s);
            telemetry.update();
            idle();
        }
    }

    /** Shows instructions; returns true on A, false on B (skip). */
    private boolean waitForA(Run run) {
        while (opModeIsActive() && (gamepad1.a || gamepad1.b)) idle(); // release buttons first
        while (opModeIsActive()) {
            follower.updatePose();
            telemetry.addLine("NEXT: " + run.label + " at " + (int) (POWER * 100) + "% for " + DRIVE_SECONDS + " s");
            telemetry.addLine("Place the robot with ~3 ft clear " + run.label.toLowerCase().replace("strafe ", "to the ") + ".");
            telemetry.addLine("Press A to run, B to skip.");
            for (String s : summary) telemetry.addLine(s);
            telemetry.update();
            if (gamepad1.a) return true;
            if (gamepad1.b) return false;
            idle();
        }
        return false;
    }

    private void doRun(Run run) {
        // Baselines
        follower.setPose(new Pose(0, 0, 0));
        follower.updatePose();
        follower.updatePose();
        Pose start = follower.getPose();
        double startPinTotal = follower.getTotalHeading();
        updateHub();
        double startHub = hubTotal;

        // Power pattern (same mecanum math as teleop): fl = f+s, bl = f-s, fr = f-s, br = f+s
        double f = run.fwd * POWER, s = run.right * POWER;
        double[] powers = {f + s, f - s, f - s, f + s};
        double[] velSum = new double[4];
        int samples = 0;

        ElapsedTime t = new ElapsedTime();
        for (int i = 0; i < 4; i++) motors[i].setPower(powers[i]);
        String safety = "";
        while (opModeIsActive() && t.seconds() < DRIVE_SECONDS) {
            follower.updatePose();
            updateHub();
            Pose p = follower.getPose();
            if (t.seconds() > DRIVE_SECONDS / 2) { // second half = steady speed
                for (int i = 0; i < 4; i++) velSum[i] += motors[i].getVelocity();
                samples++;
            }
            if (Math.hypot(p.getX() - start.getX(), p.getY() - start.getY()) > MAX_DISTANCE_IN) { safety = " (safety stop)"; break; }
            if (gamepad1.b) { safety = " (stopped by B)"; break; }
            telemetry.addData("Running", run.label + "  %.1f s", t.seconds());
            telemetry.update();
        }

        // Cut power (BRAKE) and time how long each wheel takes to stop
        Pose atCut = follower.getPose();
        double pinAtCut = follower.getTotalHeading();
        for (DcMotorEx m : motors) m.setPower(0);
        double[] stopTime = {-1, -1, -1, -1};
        ElapsedTime st = new ElapsedTime();
        while (opModeIsActive() && st.seconds() < 1.5) {
            follower.updatePose();
            updateHub();
            for (int i = 0; i < 4; i++) {
                if (stopTime[i] < 0 && Math.abs(motors[i].getVelocity()) < STOPPED_TICKS_PER_SEC) stopTime[i] = st.seconds();
            }
            if (stopTime[0] >= 0 && stopTime[1] >= 0 && stopTime[2] >= 0 && stopTime[3] >= 0 && st.seconds() > 0.5) break;
        }

        // Results in the robot's starting frame (+fwd, +left)
        Pose end = follower.getPose();
        double gx = end.getX() - start.getX(), gy = end.getY() - start.getY(), h0 = start.getHeading();
        double dx = gx * Math.cos(h0) + gy * Math.sin(h0);
        double dy = -gx * Math.sin(h0) + gy * Math.cos(h0);
        double dhPin = Math.toDegrees(follower.getTotalHeading() - startPinTotal);
        double dhHub = hubTotal - startHub;
        double dhStop = Math.toDegrees(follower.getTotalHeading() - pinAtCut);
        double[] vel = new double[4];
        boolean encoders = false;
        for (int i = 0; i < 4; i++) {
            vel[i] = samples > 0 ? velSum[i] / samples : 0;
            if (Math.abs(vel[i]) > 50) encoders = true;
        }

        summary.add(String.format("== %s%s ==", run.label, safety));
        summary.add(String.format(" Pinpoint saw: fwd %+.1f  left %+.1f in  turn %+.1f deg (hub %s)",
                dx, dy, dhPin, hubImu == null ? "n/a" : String.format("%+.1f", dhHub)));
        summary.add(" " + directionVerdict(run, dx, dy));
        if (hubImu != null && Math.abs(dhHub) > 3 && Math.signum(dhHub) != Math.signum(dhPin)) {
            summary.add(" HEADING: Pinpoint and hub IMU DISAGREE in direction -> Pinpoint heading inverted");
        }
        summary.add(" " + turnVerdict(dx, dy, dhHub, dhPin));
        if (encoders) {
            summary.add(String.format(" Wheel ticks/s: fl %.0f bl %.0f fr %.0f br %.0f",
                    vel[0], vel[1], vel[2], vel[3]));
            double left = (Math.abs(vel[0]) + Math.abs(vel[1])) / 2, right = (Math.abs(vel[2]) + Math.abs(vel[3])) / 2;
            summary.add(String.format(" Left side %.0f%% of right side speed%s", 100 * left / right,
                    Math.abs(left - right) / right > 0.05 ? "  << IMBALANCE (mechanical)" : "  (OK, within 5%)"));
            summary.add(String.format(" Stop time s: fl %.2f bl %.2f fr %.2f br %.2f | turned %+.1f deg while stopping",
                    stopTime[0], stopTime[1], stopTime[2], stopTime[3], dhStop));
        } else {
            summary.add(" Drive encoders read 0 -> wheel speed/stop checks unavailable (plug in encoder cables)");
            summary.add(String.format(" Turned %+.1f deg while stopping", dhStop));
        }
    }

    /** Did the Pinpoint report the robot moving in the direction the wheels pushed it? */
    private String directionVerdict(Run run, double dx, double dy) {
        double moved = Math.hypot(dx, dy);
        if (moved < 3) return "Pinpoint barely moved (<3 in): pods not touching the floor / unplugged?";
        if (run.fwd != 0) {
            if (Math.abs(dy) > Math.abs(dx)) return "FAIL: drove " + run.label + " but Pinpoint says SIDEWAYS -> X/Y pod ports swapped";
            if (Math.signum(dx) != run.fwd) return "FAIL: drove " + run.label + " but Pinpoint says the OPPOSITE -> flip forwardEncoderDirection";
            return "OK: Pinpoint direction matches (if the robot visibly went " + run.label + ")";
        }
        // Strafe: right = NEGATIVE y in Pedro
        int expectedSign = -run.right;
        if (Math.abs(dx) > Math.abs(dy)) return "FAIL: strafed but Pinpoint says FORWARD/BACK -> X/Y pod ports swapped";
        if (Math.signum(dy) != expectedSign) {
            return "FAIL: wheels strafed " + (run.right > 0 ? "RIGHT" : "LEFT") + " but Pinpoint says "
                    + (dy > 0 ? "LEFT" : "RIGHT") + " -> flip strafeEncoderDirection (this causes the runaway)";
        }
        return "OK: Pinpoint direction matches (if the robot visibly went " + run.label + ")";
    }

    /** How much the robot turned on its own with equal wheel power. */
    private String turnVerdict(double dx, double dy, double dhHub, double dhPin) {
        double turn = hubImu != null ? dhHub : dhPin;
        double feet = Math.max(Math.hypot(dx, dy) / 12.0, 0.5);
        double perFoot = turn / feet;
        String dir = turn > 0 ? "LEFT" : "RIGHT";
        if (Math.abs(perFoot) < 1.5) return String.format("Self-turn %.1f deg/ft: negligible", Math.abs(perFoot));
        return String.format("Self-turn %.1f deg/ft to the %s with EQUAL power -> mechanical drift (heading PID can correct)",
                Math.abs(perFoot), dir);
    }

    private void updateHub() {
        if (hubImu == null) return;
        double yaw = hubImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        hubTotal += AngleUnit.normalizeDegrees(yaw - hubPrev);
        hubPrev = yaw;
    }

    private void stopAll() {
        for (DcMotorEx m : motors) m.setPower(0);
    }
}
