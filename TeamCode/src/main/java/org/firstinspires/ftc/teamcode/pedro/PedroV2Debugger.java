package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * Pedro v2 debugger. Built to find out why the robot turns sideways and runs during
 * Pedro paths, and why one wheel looks slower than the others.
 *
 * It checks the robot in two ways:
 *  1. LOCALIZER CHECK (robot still, motors off; you push it by hand).
 *     D-Pad Right steps through 4 guided checks. Each check prints PASS / FAIL and what to change:
 *       PUSH FORWARD -> catches swapped pod cables and a reversed forward encoder
 *       PUSH LEFT    -> catches a reversed strafe encoder
 *       ROTATE CCW   -> compares Pinpoint heading with the Control Hub IMU (catches inverted heading)
 *       ROTATE 180   -> catches wrong pod offsets (forwardPodY / strafePodX)
 *  2. FOLLOWER CHECK (A drives the 24 in test path).
 *     Shows Pedro's errors, its correction vectors, each wheel's power, speed and current, and
 *     splits the wheel powers into forward / strafe / turn so you can see whether a slow wheel was
 *     COMMANDED slow by Pedro or is MECHANICALLY slow. A watchdog stops the robot when the errors
 *     run away, and freezes a snapshot of what it saw.
 *
 * Controls (gamepad 1):
 *   A          drive forward 24 in          X          drive back to (0, 0)
 *   Y          reset pose to (0,0,0)        B          emergency stop
 *   D-Pad Right  next localizer check       D-Pad Left switch telemetry page
 *   D-Pad Up     single-wheel spin test (25% power, wheels off the ground)
 */
@TeleOp(name = "Pedro v2 Full Debugger", group = "Pedro")
public class PedroV2Debugger extends OpMode {

    // ---------------- settings ----------------
    /** Watchdog: stop the path if the robot is this far (in) off the path. */
    public static double MAX_TRANSLATIONAL_ERROR_IN = 8.0;
    /** Watchdog: stop the path if the heading is this far (deg) off. */
    public static double MAX_HEADING_ERROR_DEG = 35.0;
    /** goBILDA 4-bar pod: 19.894 ticks per mm. */
    private static final double POD_TICKS_PER_INCH = 19.89436789 * 25.4;
    /** A wheel is "slow" if it turns this fraction (or less) of the median speed-per-power. */
    private static final double SLOW_WHEEL_RATIO = 0.75;

    // ---------------- hardware ----------------
    private Follower follower;
    private PathChain forward24;
    private PathChain returnOrigin;

    private GoBildaPinpointDriver pinpoint;
    private IMU hubImu;
    private DcMotorEx lf, rf, lr, rr;
    private DcMotorEx[] motors;
    private static final String[] WHEEL_NAMES = {"fl", "bl", "fr", "br"};

    // ---------------- state ----------------
    private enum TestState { TRACKING_ONLY, FOLLOWING_FORWARD, FOLLOWING_RETURN, MOTOR_ISOLATION, ABORTED }
    private enum Check { OFF, PUSH_FORWARD, PUSH_LEFT, ROTATE_CCW, ROTATE_180 }

    private TestState currentState = TestState.TRACKING_ONLY;
    private Check check = Check.OFF;
    private int page = 0; // 0 = localizer page, 1 = follower / motor page
    private int activeWheel = 0;

    // baselines (set by Y and by every new localizer check)
    private Pose basePose = new Pose(0, 0, 0);
    private double basePedroTotalHeading = 0;
    private double hubYawTotalDeg = 0, hubYawPrevDeg = 0, baseHubYawDeg = 0;
    private int baseEncX = 0, baseEncY = 0;

    // follower diagnostics
    private final ElapsedTime pathTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs = 0;
    private double maxTransErr = 0, maxHeadErrDeg = 0;
    private String startSnapshot = null;   // what Pedro commanded ~0.25 s into the path
    private String abortSnapshot = null;   // why the watchdog stopped the robot

    // edge detection
    private final boolean[] last = new boolean[8];

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        forward24 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0)))
                .setConstantHeadingInterpolation(0)
                .build();
        returnOrigin = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24, 0, 0), new Pose(0, 0, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

        // Same motors Pedro uses (names and directions come from Constants, so they always match)
        try {
            lf = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftFrontMotorName);
            lr = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftRearMotorName);
            rf = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightFrontMotorName);
            rr = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightRearMotorName);
            lf.setDirection(Constants.driveConstants.leftFrontMotorDirection);
            lr.setDirection(Constants.driveConstants.leftRearMotorDirection);
            rf.setDirection(Constants.driveConstants.rightFrontMotorDirection);
            rr.setDirection(Constants.driveConstants.rightRearMotorDirection);
            motors = new DcMotorEx[]{lf, lr, rf, rr}; // order matches WHEEL_NAMES
        } catch (Exception e) {
            telemetry.addLine("Warning: drive motor names did not match; motor checks disabled.");
        }

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConstants.hardwareMapName);
        } catch (Exception e) {
            telemetry.addLine("Warning: Pinpoint '" + Constants.localizerConstants.hardwareMapName + "' not found.");
        }

        // Control Hub IMU = independent heading reference (the one the working teleop uses)
        try {
            hubImu = hardwareMap.get(IMU.class, "imu");
            hubImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            hubImu.resetYaw();
        } catch (Exception e) {
            hubImu = null;
            telemetry.addLine("Warning: Control Hub IMU 'imu' not found; heading cross-check disabled.");
        }

        telemetry.addLine("=== PEDRO V2 DEBUGGER ===");
        telemetry.addLine("Keep the robot STILL until the Pinpoint says READY.");
        telemetry.addLine("A fwd 24in | X return | Y reset | B stop");
        telemetry.addLine("DPad Right: next localizer check | DPad Left: page");
        telemetry.addLine("DPad Up: single-wheel test (wheels off ground)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (pinpoint != null) {
            pinpoint.update();
            telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
            telemetry.addLine("Wait for READY before pressing PLAY.");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        resetBaselines();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // Buttons BEFORE follower.update(): Pedro only computes its path errors inside update(),
        // so reading them in the same loop a path was started (before update) returns null.
        handleButtons();

        // Keep the localizer updating in every state; only drive in path states
        if (currentState == TestState.MOTOR_ISOLATION) {
            follower.updatePose();
        } else {
            follower.update();
        }
        updateHubYaw();

        if (currentState == TestState.FOLLOWING_FORWARD || currentState == TestState.FOLLOWING_RETURN) {
            followerWatchdog();
        }

        telemetry.addData("State", "%s   page %d/2   loop %.0f ms", currentState, page + 1, loopMs);
        if (abortSnapshot != null) {
            telemetry.addLine("!!! WATCHDOG STOPPED THE ROBOT !!!");
            telemetry.addLine(abortSnapshot);
        }
        // A telemetry bug should show on screen, not crash the OpMode mid-test
        try {
            if (page == 0) {
                localizerPage();
            } else {
                followerPage();
            }
        } catch (RuntimeException e) {
            StackTraceElement top = e.getStackTrace().length > 0 ? e.getStackTrace()[0] : null;
            telemetry.addLine("TELEMETRY ERROR (robot still safe): " + e);
            if (top != null) telemetry.addLine("at " + top);
        }
        telemetry.update();
    }

    // =====================================================================
    // Buttons
    // =====================================================================
    private void handleButtons() {
        if (pressed(0, gamepad1.a)) {
            startPath(forward24, TestState.FOLLOWING_FORWARD);
        }
        if (pressed(1, gamepad1.x)) {
            startPath(returnOrigin, TestState.FOLLOWING_RETURN);
        }
        if (pressed(2, gamepad1.y)) {
            follower.breakFollowing();
            follower.setPose(new Pose(0, 0, 0));
            currentState = TestState.TRACKING_ONLY;
            abortSnapshot = null;
            startSnapshot = null;
            resetBaselines();
        }
        if (pressed(3, gamepad1.dpad_up) && motors != null) {
            follower.breakFollowing();
            currentState = TestState.MOTOR_ISOLATION;
            activeWheel = (activeWheel + 1) % 5; // 1-4 = wheels, 0 = off
            testSingleMotor(activeWheel);
            page = 1;
        }
        if (pressed(4, gamepad1.dpad_right)) {
            Check[] all = Check.values();
            check = all[(check.ordinal() + 1) % all.length];
            resetBaselines();
            page = 0;
        }
        if (pressed(5, gamepad1.dpad_left)) {
            page = 1 - page;
        }
        if (gamepad1.b) {
            follower.breakFollowing();
            stopAllMotors();
            if (currentState != TestState.ABORTED) currentState = TestState.TRACKING_ONLY;
        }
        if ((currentState == TestState.FOLLOWING_FORWARD || currentState == TestState.FOLLOWING_RETURN)
                && !follower.isBusy()) {
            currentState = TestState.TRACKING_ONLY;
        }
    }

    private void startPath(PathChain path, TestState state) {
        abortSnapshot = null;
        startSnapshot = null;
        maxTransErr = 0;
        maxHeadErrDeg = 0;
        resetBaselines();
        follower.followPath(path);
        currentState = state;
        pathTimer.reset();
        page = 1;
    }

    private boolean pressed(int i, boolean now) {
        boolean p = now && !last[i];
        last[i] = now;
        return p;
    }

    // =====================================================================
    // Baselines and heading reference
    // =====================================================================
    private void resetBaselines() {
        basePose = follower.getPose();
        basePedroTotalHeading = follower.getTotalHeading();
        baseHubYawDeg = hubYawTotalDeg;
        if (pinpoint != null) {
            baseEncX = pinpoint.getEncoderX();
            baseEncY = pinpoint.getEncoderY();
        }
    }

    /** Unwrapped Control Hub IMU yaw, so a full turn reads 360 instead of jumping at 180. */
    private void updateHubYaw() {
        if (hubImu == null) return;
        double yaw = hubImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        hubYawTotalDeg += AngleUnit.normalizeDegrees(yaw - hubYawPrevDeg);
        hubYawPrevDeg = yaw;
    }

    /** Movement since the baseline, in the robot frame at the baseline (x = forward, y = left). */
    private double[] robotFrameDelta() {
        Pose p = follower.getPose();
        double gx = p.getX() - basePose.getX();
        double gy = p.getY() - basePose.getY();
        double h0 = basePose.getHeading();
        double dx = gx * Math.cos(h0) + gy * Math.sin(h0);
        double dy = -gx * Math.sin(h0) + gy * Math.cos(h0);
        return new double[]{dx, dy};
    }

    private double pinpointHeadingDeltaDeg() {
        return Math.toDegrees(follower.getTotalHeading() - basePedroTotalHeading);
    }

    private double hubHeadingDeltaDeg() {
        return hubYawTotalDeg - baseHubYawDeg;
    }

    // =====================================================================
    // Page 1: localizer
    // =====================================================================
    private void localizerPage() {
        Pose pose = follower.getPose();
        double[] d = robotFrameDelta();
        double dhPin = pinpointHeadingDeltaDeg();
        double dhHub = hubHeadingDeltaDeg();

        telemetry.addLine("--- PINPOINT HEALTH ---");
        if (pinpoint != null) {
            telemetry.addData("Status", "%s  (%.0f Hz)", pinpoint.getDeviceStatus(), pinpoint.getFrequency());
            try {
                // Older Pinpoint firmware (v1/v2) has no roll/pitch output and throws here
                double roll = pinpoint.getRoll(AngleUnit.DEGREES);
                double pitch = pinpoint.getPitch(AngleUnit.DEGREES);
                telemetry.addData("Roll | Pitch", "%.1f | %.1f deg  %s", roll, pitch, mountingVerdict(roll, pitch));
            } catch (RuntimeException e) {
                telemetry.addData("Roll | Pitch", "n/a (old Pinpoint firmware) - check it is flat, label up");
            }
            telemetry.addData("Offsets in use", "fwdPodY %.2f  strafePodX %.2f in",
                    pinpoint.getXOffset(DistanceUnit.INCH), pinpoint.getYOffset(DistanceUnit.INCH));
            int dEx = pinpoint.getEncoderX() - baseEncX;
            int dEy = pinpoint.getEncoderY() - baseEncY;
            telemetry.addData("Raw pods since reset", "X(fwd) %d (%.1f in) | Y(strafe) %d (%.1f in)",
                    dEx, dEx / POD_TICKS_PER_INCH, dEy, dEy / POD_TICKS_PER_INCH);
        } else {
            telemetry.addLine("Pinpoint not found");
        }

        telemetry.addLine("--- POSE (Pedro) ---");
        telemetry.addData("X | Y | H", "%.2f in | %.2f in | %.1f deg",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Moved (robot frame)", "fwd %.2f  left %.2f in", d[0], d[1]);
        telemetry.addData("Turned", "Pinpoint %+.1f  |  Hub IMU %s", dhPin,
                hubImu == null ? "n/a" : String.format("%+.1f deg", dhHub));
        telemetry.addLine(headingAgreement(dhPin, dhHub));

        telemetry.addLine("--- LOCALIZER CHECK (DPad Right = next) ---");
        telemetry.addData("Check", check);
        for (String line : checkVerdict(d[0], d[1], dhPin, dhHub)) {
            telemetry.addLine(line);
        }
    }

    private String mountingVerdict(double roll, double pitch) {
        if (Math.abs(roll) > 90 || Math.abs(pitch) > 90) return "<< UPSIDE DOWN? heading may be inverted";
        if (Math.abs(roll) > 5 || Math.abs(pitch) > 5) return "<< not flat, mount it level";
        return "OK";
    }

    private String headingAgreement(double dhPin, double dhHub) {
        if (hubImu == null) return "Heading cross-check: no hub IMU";
        if (Math.abs(dhHub) < 10) return "Heading cross-check: rotate the robot >10 deg to compare";
        if (Math.signum(dhPin) != Math.signum(dhHub)) {
            return "Heading cross-check: FAIL - Pinpoint heading is INVERTED vs hub IMU";
        }
        double ratio = dhPin / dhHub;
        if (ratio < 0.9 || ratio > 1.1) {
            return String.format("Heading cross-check: WARN - Pinpoint/hub ratio %.2f (expect ~1.00)", ratio);
        }
        return String.format("Heading cross-check: OK (ratio %.2f)", ratio);
    }

    private String[] checkVerdict(double dx, double dy, double dhPin, double dhHub) {
        double moved = Math.hypot(dx, dy);
        int dEx = pinpoint == null ? 0 : pinpoint.getEncoderX() - baseEncX;
        int dEy = pinpoint == null ? 0 : pinpoint.getEncoderY() - baseEncY;
        switch (check) {
            case OFF:
                return new String[]{"Press DPad Right to start. Robot on the floor, push by hand."};

            case PUSH_FORWARD:
                if (moved < 6 && Math.abs(dEx) + Math.abs(dEy) < 6 * POD_TICKS_PER_INCH) {
                    return new String[]{"Push the robot STRAIGHT FORWARD about 24 in.",
                            "(Use a tape line; keep it from turning.)"};
                }
                if (Math.abs(dEy) > Math.abs(dEx)) {
                    return new String[]{"FAIL: forward push moved the Y (strafe) pod, not X.",
                            "-> Pod cables SWAPPED on the Pinpoint X/Y ports",
                            "   (or a pod is mounted turned 90 deg)."};
                }
                if (Math.abs(dy) > Math.abs(dx)) {
                    return new String[]{"FAIL: forward push shows up as sideways in Pedro.",
                            "-> Check pod ports and that heading started at 0 (press Y)."};
                }
                if (dx < 0) {
                    return new String[]{"FAIL: pushed forward but X went DOWN.",
                            "-> Flip forwardEncoderDirection in Constants."};
                }
                return new String[]{String.format("PASS: forward %.1f in (sideways %.1f).", dx, dy),
                        "Compare with the tape: if off by >2%, run Forward Tuner."};

            case PUSH_LEFT:
                if (moved < 6 && Math.abs(dEx) + Math.abs(dEy) < 6 * POD_TICKS_PER_INCH) {
                    return new String[]{"Push the robot STRAIGHT LEFT (sideways) about 24 in."};
                }
                if (Math.abs(dEx) > Math.abs(dEy)) {
                    return new String[]{"FAIL: sideways push moved the X (forward) pod, not Y.",
                            "-> Pod cables SWAPPED on the Pinpoint X/Y ports."};
                }
                if (dy < 0) {
                    return new String[]{"FAIL: pushed LEFT but Y went DOWN.",
                            "-> Flip strafeEncoderDirection in Constants."};
                }
                return new String[]{String.format("PASS: left %.1f in (forward %.1f).", dy, dx),
                        "Compare with the tape: if off by >2%, run Lateral Tuner."};

            case ROTATE_CCW:
                if (hubImu != null && Math.abs(dhHub) < 30) {
                    return new String[]{"Rotate the robot COUNTER-CLOCKWISE (to the left) ~90 deg."};
                }
                if (hubImu == null && Math.abs(dhPin) < 30) {
                    return new String[]{"Rotate the robot COUNTER-CLOCKWISE ~90 deg (no hub IMU found)."};
                }
                if (hubImu != null && dhHub < 0) {
                    return new String[]{"You turned CLOCKWISE (hub IMU says so).",
                            "Press Y to reset, then turn LEFT."};
                }
                if (dhPin < 0) {
                    return new String[]{"FAIL: turning LEFT made Pinpoint heading go DOWN.",
                            "-> Heading is INVERTED. Pinpoint must be flat, label UP.",
                            "   This makes Pedro turn sideways and run away."};
                }
                return new String[]{String.format("PASS: Pinpoint %+.1f deg vs hub %s.", dhPin,
                        hubImu == null ? "n/a" : String.format("%+.1f", dhHub))};

            case ROTATE_180:
                if (Math.abs(dhPin) < 170) {
                    return new String[]{"Spin the robot IN PLACE 180 deg (either way),",
                            "keeping its center on one spot. Now: " + String.format("%.0f deg", dhPin)};
                }
                // A wrong offset makes the computed center swing in a circle;
                // after 180 deg it is off by about 2x the offset error.
                double offsetErr = moved / 2.0;
                if (moved > 2.0) {
                    return new String[]{String.format("WARN: center drifted %.1f in (fwd %.1f, left %.1f).", moved, dx, dy),
                            String.format("-> Pod offsets off by ~%.1f in. Re-measure", offsetErr),
                            "   forwardPodY (left +) and strafePodX (forward +)."};
                }
                return new String[]{String.format("PASS: center drift %.1f in after %.0f deg.", moved, dhPin)};
        }
        return new String[]{""};
    }

    // =====================================================================
    // Page 2: follower and motors
    // =====================================================================
    private void followerPage() {
        Pose pose = follower.getPose();
        telemetry.addData("Pose", "X %.2f  Y %.2f  H %.1f deg", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addLine(headingAgreement(pinpointHeadingDeltaDeg(), hubHeadingDeltaDeg()));

        if (follower.isBusy()) {
            Vector te = follower.getTranslationalError();
            telemetry.addLine("--- PEDRO ERRORS ---");
            telemetry.addData("Translational err", "x %.2f  y %.2f  |%.2f| in", te.getXComponent(), te.getYComponent(), te.getMagnitude());
            telemetry.addData("Heading err", "%.1f deg", Math.toDegrees(follower.getHeadingError()));
            telemetry.addData("Drive err", "%.2f", follower.getDriveError());
            telemetry.addData("Path done", "%.0f%%", 100 * follower.getPathCompletion());
            telemetry.addLine("--- PEDRO COMMANDS (field frame) ---");
            telemetry.addData("Drive vec", vec(follower.getDriveVector()));
            telemetry.addData("Corrective vec", vec(follower.getCorrectiveVector()));
            telemetry.addData("Heading vec", vec(follower.getHeadingVector()));
        }
        telemetry.addData("Max err this run", "trans %.2f in  heading %.1f deg", maxTransErr, maxHeadErrDeg);
        if (startSnapshot != null) {
            telemetry.addLine("--- START OF PATH (0.25 s) ---");
            telemetry.addLine(startSnapshot);
        }

        if (motors != null) {
            if (currentState == TestState.MOTOR_ISOLATION) {
                telemetry.addLine("Single-wheel test: the one powered wheel must spin FORWARD"
                        + " and show positive ticks/s. DPad Up = next wheel.");
            }
            telemetry.addLine("--- WHEELS (power | ticks/s | amps) ---");
            for (int i = 0; i < 4; i++) {
                telemetry.addData(WHEEL_NAMES[i], "%+.2f | %+6.0f | %.2f A",
                        motors[i].getPower(), motors[i].getVelocity(), motors[i].getCurrent(CurrentUnit.AMPS));
            }
            telemetry.addLine(powerBreakdown());
            telemetry.addLine(slowWheelVerdict());
        }
    }

    private String vec(Vector v) {
        return String.format("mag %.2f  dir %.0f deg", v.getMagnitude(), Math.toDegrees(v.getTheta()));
    }

    /**
     * Splits the four wheel powers into the motions they make.
     * fl = f + s + t,  bl = f - s + t,  fr = f - s - t,  br = f + s - t   (s = strafe right, t = turn right)
     * If one wheel looks slow and strafe/turn are not ~0, Pedro is COMMANDING a correction.
     */
    private String powerBreakdown() {
        double fl = lf.getPower(), bl = lr.getPower(), fr = rf.getPower(), br = rr.getPower();
        double f = (fl + bl + fr + br) / 4;
        double s = (fl - bl - fr + br) / 4;
        double t = (fl + bl - fr - br) / 4;
        return String.format("Commanded: fwd %+.2f  strafeR %+.2f  turnR %+.2f", f, s, t);
    }

    /** Compares how fast each wheel turns for the power it was given. */
    private String slowWheelVerdict() {
        double[] ratio = new double[4];
        int n = 0;
        boolean anyEncoder = false;
        for (int i = 0; i < 4; i++) {
            double p = motors[i].getPower();
            double v = motors[i].getVelocity();
            if (Math.abs(v) > 1) anyEncoder = true;
            ratio[i] = Math.abs(p) > 0.15 ? Math.abs(v / p) : Double.NaN;
            if (!Double.isNaN(ratio[i])) n++;
        }
        if (n < 2) return "Wheel speed check: needs >15% power on 2+ wheels";
        if (!anyEncoder) return "Wheel speed check: no drive encoder counts (cables unplugged?)";
        double[] valid = new double[n];
        int k = 0;
        for (double r : ratio) if (!Double.isNaN(r)) valid[k++] = r;
        Arrays.sort(valid);
        double median = valid[n / 2];
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            if (!Double.isNaN(ratio[i]) && ratio[i] < SLOW_WHEEL_RATIO * median) {
                sb.append(WHEEL_NAMES[i]).append(String.format(" %.0f%% ", 100 * ratio[i] / median));
            }
        }
        if (sb.length() == 0) return "Wheel speed check: all wheels match their power (not mechanical)";
        return "MECHANICAL? slow for its power: " + sb + "-> run Motor Balance Test";
    }

    // =====================================================================
    // Watchdog
    // =====================================================================
    private void followerWatchdog() {
        double trans = follower.getTranslationalError().getMagnitude();
        double headDeg = Math.abs(Math.toDegrees(follower.getHeadingError()));
        maxTransErr = Math.max(maxTransErr, trans);
        maxHeadErrDeg = Math.max(maxHeadErrDeg, headDeg);

        if (startSnapshot == null && pathTimer.seconds() > 0.25) {
            startSnapshot = String.format("trans err %.2f in, heading err %.1f deg\n%s",
                    trans, Math.toDegrees(follower.getHeadingError()),
                    motors != null ? powerBreakdown() : "");
        }

        String reason = null;
        if (trans > MAX_TRANSLATIONAL_ERROR_IN) reason = String.format("off path by %.1f in", trans);
        if (headDeg > MAX_HEADING_ERROR_DEG) reason = String.format("heading off by %.0f deg", headDeg);
        if (follower.isLocalizationNAN()) reason = "localizer returned NaN";
        if (reason == null) return;

        Pose p = follower.getPose();
        double dhPin = pinpointHeadingDeltaDeg();
        double dhHub = hubHeadingDeltaDeg();
        abortSnapshot = String.format("Reason: %s at %.2f s\nPose X %.1f Y %.1f H %.0f deg\n" +
                        "Turned: Pinpoint %+.0f vs hub %+.0f deg\n%s",
                reason, pathTimer.seconds(), p.getX(), p.getY(), Math.toDegrees(p.getHeading()),
                dhPin, dhHub, likelyCause(dhPin, dhHub));
        follower.breakFollowing();
        stopAllMotors();
        currentState = TestState.ABORTED;
    }

    private String likelyCause(double dhPin, double dhHub) {
        if (hubImu != null && Math.abs(dhHub) > 10 && Math.signum(dhPin) != Math.signum(dhHub)) {
            return "Likely: Pinpoint heading INVERTED (see page 1)";
        }
        if (hubImu != null && Math.abs(dhHub) > 10 && Math.abs(dhPin) < 0.5 * Math.abs(dhHub)) {
            return "Likely: Pinpoint heading not tracking the real turn";
        }
        return "Run the 4 localizer checks on page 1 (DPad Right)";
    }

    // =====================================================================
    // Single-wheel test
    // =====================================================================
    private void testSingleMotor(int wheelIndex) {
        stopAllMotors();
        double testPower = 0.25;
        switch (wheelIndex) {
            case 1: lf.setPower(testPower); break;
            case 2: rf.setPower(testPower); break;
            case 3: lr.setPower(testPower); break;
            case 4: rr.setPower(testPower); break;
            default: currentState = TestState.TRACKING_ONLY; break;
        }
        String[] names = {"off", "FRONT LEFT", "FRONT RIGHT", "REAR LEFT", "REAR RIGHT"};
        telemetry.addLine("Testing: " + names[wheelIndex] + " - should spin FORWARD, ticks/s positive");
    }

    private void stopAllMotors() {
        if (motors == null) return;
        for (DcMotorEx m : motors) m.setPower(0);
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        stopAllMotors();
    }
}
