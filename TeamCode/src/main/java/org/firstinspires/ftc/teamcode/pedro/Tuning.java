package org.firstinspires.ftc.teamcode.pedro;

import android.annotation.SuppressLint;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Pedro Pathing 2.1.2 tuning OpModes (group "Pedro Tuning" on the Driver Hub).
 * No extra libraries needed: results show on the Driver Hub, and PID gains are edited live
 * with the gamepad, then copied into {@link Constants}.
 *
 * RUN THEM IN THIS ORDER. Each step depends on the ones before it.
 *  0. "Pedro v2 Full Debugger": pass all 4 localizer checks first.
 *  1. Localizer:  Localization Test, Forward Tuner, Lateral Tuner, Turn Tuner
 *  2. Robot:      weigh the robot (kg) -> Constants.followerConstants.mass(...)
 *                 Forward Velocity Tuner, Lateral Velocity Tuner            -> driveConstants
 *                 Forward Zero Power Accel Tuner, Lateral Zero Power Accel Tuner -> followerConstants
 *  3. PIDs:       Translational Tuner, Heading Tuner, Drive Tuner, Centripetal Tuner
 *  4. Check:      Line Test (all PIDs on)
 *
 * Driving tuners (velocity, zero power accel) need a clear lane: about 6 ft ahead (or to the left).
 * B always stops the robot.
 *
 * Gain editing (PID tuners):
 *   DPad Left/Right = pick gain   DPad Up/Down = raise/lower it
 *   Bumpers = change step size    Y = next controller (Line Test only)
 * Pedro re-reads these gains every loop, so changes apply instantly. They are NOT saved:
 * copy the code line shown on screen into Constants.java.
 */
public class Tuning {
    public static final String GROUP = "Pedro Tuning";

    /** Distance you push the robot by hand in the Forward/Lateral Tuners (in). */
    public static double PUSH_DISTANCE = 48;
    /** Full turns you spin the robot by hand in the Turn Tuner. */
    public static double TURNS = 1;
    /** Distance driven at full power in the velocity tuners (in). */
    public static double VELOCITY_DISTANCE = 48;
    /** Distance driven at full power before coasting in the zero power accel tuners (in). */
    public static double ZPA_DISTANCE = 30;
    /** Length of the back-and-forth line in the path tuners (in). */
    public static double PATH_DISTANCE = 40;
    /** Size of the curve in the Centripetal Tuner (in). */
    public static double CURVE_SIZE = 24;
    /** Driving tuners stop if the robot turns more than this (deg) - usually a localizer problem. */
    public static double SAFETY_HEADING_DEG = 25;
    /** goBILDA 4-bar pod resolution in ticks per inch (19.894 ticks/mm). */
    static final double POD_TICKS_PER_INCH = 19.89436789 * 25.4;

    private Tuning() {}

    /** Velocity in the robot frame: [forward, left]. */
    static double[] robotVelocity(Follower f) {
        Vector v = f.getVelocity();
        double h = f.getPose().getHeading();
        double vx = v.getXComponent(), vy = v.getYComponent();
        return new double[]{vx * Math.cos(h) + vy * Math.sin(h), -vx * Math.sin(h) + vy * Math.cos(h)};
    }

    static double average(List<Double> list) {
        double sum = 0;
        for (double d : list) sum += d;
        return list.isEmpty() ? 0 : sum / list.size();
    }

    // =========================================================================================
    // Shared base
    // =========================================================================================
    public abstract static class Base extends OpMode {
        protected Follower follower;
        private final boolean[] last = new boolean[12];

        @Override
        public void init() {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(0, 0, 0));
            follower.update();
            initTuner();
        }

        protected void initTuner() {}

        protected boolean pressed(int i, boolean now) {
            boolean p = now && !last[i];
            last[i] = now;
            return p;
        }

        @Override
        public void stop() {
            follower.breakFollowing();
        }
    }

    // =========================================================================================
    // 1. LOCALIZER
    // =========================================================================================

    /** Drive around with the sticks and watch the pose. Y resets to (0,0,0). */
    @TeleOp(name = "Localization Test", group = GROUP)
    public static class LocalizationTest extends Base {
        @Override
        protected void initTuner() {
            telemetry.addLine("Drive with the sticks (robot-centric). Y = reset pose.");
            telemetry.addLine("Forward -> X up, left -> Y up, turn left -> heading up.");
            telemetry.update();
        }

        @Override
        public void start() {
            follower.startTeleopDrive(true);
        }

        @Override
        public void loop() {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            if (pressed(0, gamepad1.y)) follower.setPose(new Pose(0, 0, 0));

            Pose p = follower.getPose();
            double[] v = robotVelocity(follower);
            telemetry.addData("X | Y", "%.2f | %.2f in", p.getX(), p.getY());
            telemetry.addData("Heading", "%.1f deg (total %.1f)", Math.toDegrees(p.getHeading()), Math.toDegrees(follower.getTotalHeading()));
            telemetry.addData("Velocity", "fwd %.1f  left %.1f in/s", v[0], v[1]);
            telemetry.addData("Localizer NaN", follower.isLocalizationNAN());
            telemetry.update();
        }
    }

    /** Push the robot straight forward PUSH_DISTANCE inches; checks the pod resolution. */
    @TeleOp(name = "Forward Tuner", group = GROUP)
    public static class ForwardTuner extends Base {
        @Override
        public void loop() {
            follower.update();
            showPushResult(telemetry, "FORWARD", follower.getPose().getX());
        }
    }

    /** Push the robot straight left PUSH_DISTANCE inches; checks the pod resolution. */
    @TeleOp(name = "Lateral Tuner", group = GROUP)
    public static class LateralTuner extends Base {
        @Override
        public void loop() {
            follower.update();
            showPushResult(telemetry, "LEFT", follower.getPose().getY());
        }
    }

    @SuppressLint({"NewApi", "DefaultLocale"})
    static void showPushResult(Telemetry telemetry, String direction, double measured) {
        double current = Constants.localizerConstants.customEncoderResolution.isPresent()
                ? Constants.localizerConstants.customEncoderResolution.getAsDouble()
                : POD_TICKS_PER_INCH;
        telemetry.addLine(String.format("Push the robot straight %s exactly %.0f in (use a tape measure).", direction, PUSH_DISTANCE));
        telemetry.addData("Measured", "%.2f in", measured);
        if (Math.abs(measured) < 0.5 * PUSH_DISTANCE) {
            telemetry.addLine("Keep pushing...");
        } else if (measured < 0) {
            telemetry.addLine("NEGATIVE: encoder direction is reversed. Fix that first (see Debugger).");
        } else {
            double errPct = 100 * (measured - PUSH_DISTANCE) / PUSH_DISTANCE;
            telemetry.addData("Error", "%+.1f%%", errPct);
            if (Math.abs(errPct) <= 2) {
                telemetry.addLine("GOOD (within 2%). No change needed.");
            } else {
                telemetry.addLine("Repeat 3 times. If always off the same way, add to localizerConstants:");
                telemetry.addLine(String.format(".customEncoderResolution(%.3f)  // ticks per inch", current * measured / PUSH_DISTANCE));
                telemetry.addLine("If only ONE direction is off, check that pod's wheel and spring instead.");
            }
        }
        telemetry.update();
    }

    /** Spin the robot by hand TURNS full turns; checks the Pinpoint heading scale. */
    @TeleOp(name = "Turn Tuner", group = GROUP)
    public static class TurnTuner extends Base {
        @SuppressLint("NewApi")
        @Override
        public void loop() {
            follower.update();
            double measured = Math.toDegrees(follower.getTotalHeading());
            double target = 360 * TURNS;
            telemetry.addLine(String.format("Spin the robot COUNTER-CLOCKWISE exactly %.0f full turn(s).", TURNS));
            telemetry.addLine("Line it up with a wall or tile seam to start and stop.");
            telemetry.addData("Measured", "%.1f deg (target %.0f)", measured, target);
            if (Math.abs(measured) > 0.5 * target) {
                if (measured < 0) {
                    telemetry.addLine("NEGATIVE while turning CCW: heading is INVERTED. Fix the Pinpoint mounting.");
                } else {
                    double errPct = 100 * (measured - target) / target;
                    telemetry.addData("Error", "%+.2f%%", errPct);
                    if (Math.abs(errPct) <= 1) {
                        telemetry.addLine("GOOD. The Pinpoint is factory calibrated; leave yawScalar alone.");
                    } else {
                        double cur = Constants.localizerConstants.yawScalar.isPresent()
                                ? Constants.localizerConstants.yawScalar.getAsDouble() : 1.0;
                        telemetry.addLine("Repeat 3 times. Only if always off, add to localizerConstants:");
                        telemetry.addLine(String.format(".yawScalar(%.4f)", cur * target / measured));
                    }
                }
            }
            telemetry.update();
        }
    }

    // =========================================================================================
    // 2. ROBOT MEASUREMENTS (the robot drives itself - clear the lane!)
    // =========================================================================================

    /** Drives at full power and measures top speed -> xVelocity / yVelocity. */
    public abstract static class VelocityTuner extends Base {
        private final boolean lateral;
        private final List<Double> samples = new ArrayList<>();
        private String result = null;
        private boolean running = false;

        protected VelocityTuner(boolean lateral) { this.lateral = lateral; }

        @Override
        protected void initTuner() {
            telemetry.addLine("The robot drives " + (lateral ? "LEFT" : "FORWARD") + " at FULL POWER for "
                    + (int) VELOCITY_DISTANCE + " in. Clear about 6 ft. B = stop.");
            telemetry.addLine("Use a fully charged battery.");
            telemetry.update();
        }

        @Override
        public void start() {
            follower.startTeleopDrive(true);
            running = true;
        }

        @Override
        public void loop() {
            if (running) {
                follower.setTeleOpDrive(lateral ? 0 : 1, lateral ? 1 : 0, 0, true);
                follower.update();
                Pose p = follower.getPose();
                double[] v = robotVelocity(follower);
                samples.add(lateral ? v[1] : v[0]);
                if (samples.size() > 10) samples.remove(0); // keep the last 10 (top speed)

                double dist = lateral ? p.getY() : p.getX();
                if (gamepad1.b) {
                    finish("Stopped by driver.");
                } else if (Math.abs(Math.toDegrees(p.getHeading())) > SAFETY_HEADING_DEG
                        && Math.abs(Math.toDegrees(p.getHeading())) < 360 - SAFETY_HEADING_DEG) {
                    finish("ABORTED: robot turned >" + (int) SAFETY_HEADING_DEG + " deg. Fix localizer / wheels first.");
                } else if (dist < -6) {
                    finish("ABORTED: moving the wrong way. Check motor directions / encoder directions.");
                } else if (dist >= VELOCITY_DISTANCE) {
                    finish(String.format("Add to driveConstants:  .%s(%.3f)", lateral ? "yVelocity" : "xVelocity", average(samples)));
                }
                telemetry.addData("Distance", "%.1f / %.0f in", dist, VELOCITY_DISTANCE);
                telemetry.addData("Velocity", "%.1f in/s", samples.isEmpty() ? 0 : samples.get(samples.size() - 1));
            } else {
                follower.updatePose();
            }
            if (result != null) {
                telemetry.addLine(result);
                telemetry.addLine("Run 3 times and average. Too low? The robot may not have hit top speed:");
                telemetry.addLine("raise Tuning.VELOCITY_DISTANCE if you have room.");
            }
            telemetry.update();
        }

        private void finish(String msg) {
            follower.setTeleOpDrive(0, 0, 0, true);
            follower.update();
            follower.breakFollowing();
            running = false;
            result = msg;
        }
    }

    @TeleOp(name = "Forward Velocity Tuner", group = GROUP)
    public static class ForwardVelocityTuner extends VelocityTuner {
        public ForwardVelocityTuner() { super(false); }
    }

    @TeleOp(name = "Lateral Velocity Tuner", group = GROUP)
    public static class LateralVelocityTuner extends VelocityTuner {
        public LateralVelocityTuner() { super(true); }
    }

    /** Drives at full power, cuts power, measures how fast it coasts to a stop. */
    public abstract static class ZeroPowerAccelTuner extends Base {
        private enum Phase { ACCELERATE, COAST, DONE }
        private final boolean lateral;
        private final List<Double> decel = new ArrayList<>();
        private final ElapsedTime dt = new ElapsedTime();
        private Phase phase = Phase.ACCELERATE;
        private double prevVel = 0;
        private String result = null;

        protected ZeroPowerAccelTuner(boolean lateral) { this.lateral = lateral; }

        @Override
        protected void initTuner() {
            telemetry.addLine("The robot drives " + (lateral ? "LEFT" : "FORWARD") + " at FULL POWER for "
                    + (int) ZPA_DISTANCE + " in, then coasts. Clear about 6 ft. B = stop.");
            telemetry.update();
        }

        @Override
        public void start() {
            follower.startTeleopDrive(false); // FLOAT: we are measuring coasting, not braking
        }

        @Override
        public void loop() {
            if (gamepad1.b && phase != Phase.DONE) {
                stopDriving();
                phase = Phase.DONE;
                result = "Stopped by driver.";
            }
            switch (phase) {
                case ACCELERATE: {
                    follower.setTeleOpDrive(lateral ? 0 : 1, lateral ? 1 : 0, 0, true);
                    follower.update();
                    Pose p = follower.getPose();
                    double dist = lateral ? p.getY() : p.getX();
                    if (Math.abs(Math.toDegrees(p.getHeading())) > SAFETY_HEADING_DEG
                            && Math.abs(Math.toDegrees(p.getHeading())) < 360 - SAFETY_HEADING_DEG) {
                        stopDriving();
                        phase = Phase.DONE;
                        result = "ABORTED: robot turned. Fix localizer / wheels first.";
                    } else if (dist >= ZPA_DISTANCE) {
                        stopDriving();
                        prevVel = vel();
                        dt.reset();
                        phase = Phase.COAST;
                    }
                    telemetry.addData("Accelerating", "%.1f / %.0f in", dist, ZPA_DISTANCE);
                    break;
                }
                case COAST: {
                    follower.updatePose();
                    double v = vel();
                    double seconds = dt.seconds();
                    dt.reset();
                    if (v > 1.0) {
                        if (seconds > 0) decel.add((v - prevVel) / seconds);
                        prevVel = v;
                    } else {
                        phase = Phase.DONE;
                        result = String.format("Add to followerConstants:  .%s(%.3f)",
                                lateral ? "lateralZeroPowerAcceleration" : "forwardZeroPowerAcceleration", average(decel));
                    }
                    telemetry.addData("Coasting", "%.1f in/s", v);
                    break;
                }
                case DONE:
                    follower.updatePose();
                    break;
            }
            if (result != null) {
                telemetry.addLine(result);
                telemetry.addLine("Should be NEGATIVE. Run 3 times and average.");
            }
            telemetry.update();
        }

        private double vel() {
            double[] v = robotVelocity(follower);
            return lateral ? v[1] : v[0];
        }

        private void stopDriving() {
            follower.setTeleOpDrive(0, 0, 0, true);
            follower.update();
            follower.breakFollowing(); // sets motors to FLOAT and zero power
        }
    }

    @TeleOp(name = "Forward Zero Power Accel Tuner", group = GROUP)
    public static class ForwardZeroPowerAccelTuner extends ZeroPowerAccelTuner {
        public ForwardZeroPowerAccelTuner() { super(false); }
    }

    @TeleOp(name = "Lateral Zero Power Accel Tuner", group = GROUP)
    public static class LateralZeroPowerAccelTuner extends ZeroPowerAccelTuner {
        public LateralZeroPowerAccelTuner() { super(true); }
    }

    // =========================================================================================
    // 3. PID TUNERS
    // =========================================================================================

    /** Edits a set of gains live with the gamepad D-Pad and bumpers. */
    static final class GainEditor {
        interface Getter { double get(); }
        interface Setter { void set(double v); }
        interface Code { String get(); }

        private static final double[] STEPS = {0.0001, 0.001, 0.01, 0.1, 1};
        final String title;
        private final String[] labels;
        private final Getter[] getters;
        private final Setter[] setters;
        private final Code code;
        private int selected = 0;
        private int step = 2;
        private final boolean[] last = new boolean[6];

        GainEditor(String title, String[] labels, Getter[] getters, Setter[] setters, Code code) {
            this.title = title;
            this.labels = labels;
            this.getters = getters;
            this.setters = setters;
            this.code = code;
        }

        @SuppressLint("DefaultLocale")
        static GainEditor pidf(String title, String method, final PIDFCoefficients c) {
            return new GainEditor(title, new String[]{"P", "I", "D", "F"},
                    new Getter[]{() -> c.P, () -> c.I, () -> c.D, () -> c.F},
                    new Setter[]{v -> c.P = v, v -> c.I = v, v -> c.D = v, v -> c.F = v},
                    () -> String.format(".%s(new PIDFCoefficients(%s, %s, %s, %s))", method, n(c.P), n(c.I), n(c.D), n(c.F)));
        }

        @SuppressLint("DefaultLocale")
        static GainEditor filtered(String title, String method, final FilteredPIDFCoefficients c) {
            return new GainEditor(title, new String[]{"P", "I", "D", "T", "F"},
                    new Getter[]{() -> c.P, () -> c.I, () -> c.D, () -> c.T, () -> c.F},
                    new Setter[]{v -> c.P = v, v -> c.I = v, v -> c.D = v, v -> c.T = v, v -> c.F = v},
                    () -> String.format(".%s(new FilteredPIDFCoefficients(%s, %s, %s, %s, %s))", method, n(c.P), n(c.I), n(c.D), n(c.T), n(c.F)));
        }

        static String n(double v) {
            return String.valueOf(Math.round(v * 1e6) / 1e6);
        }

        void handle(Gamepad g) {
            if (pressed(0, g.dpad_right)) selected = (selected + 1) % labels.length;
            if (pressed(1, g.dpad_left)) selected = (selected + labels.length - 1) % labels.length;
            if (pressed(2, g.right_bumper)) step = Math.min(step + 1, STEPS.length - 1);
            if (pressed(3, g.left_bumper)) step = Math.max(step - 1, 0);
            if (pressed(4, g.dpad_up)) setters[selected].set(getters[selected].get() + STEPS[step]);
            if (pressed(5, g.dpad_down)) setters[selected].set(Math.max(0, getters[selected].get() - STEPS[step]));
        }

        private boolean pressed(int i, boolean now) {
            boolean p = now && !last[i];
            last[i] = now;
            return p;
        }

        @SuppressLint("DefaultLocale")
        void show(Telemetry t) {
            t.addLine("--- " + title + " (step " + n(STEPS[step]) + ") ---");
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < labels.length; i++) {
                sb.append(i == selected ? "[" : " ").append(labels[i]).append("=").append(n(getters[i].get()))
                        .append(i == selected ? "]" : " ").append(" ");
            }
            t.addLine(sb.toString());
            t.addLine("Copy into followerConstants:");
            t.addLine(code.get());
        }
    }

    /** Drives a path back and forth while you edit gains. */
    public abstract static class PathTuner extends Base {
        protected PathChain forward, backward;
        private boolean goingForward = true;
        private boolean stopped = false;
        protected final List<GainEditor> editors = new ArrayList<>();
        private int editorIndex = 0;

        /** Turn on only the controllers this tuner is about. */
        protected abstract void setControllers();
        protected abstract void buildPaths();
        protected abstract String instructions();
        /** False for tuners where the robot only holds still (no drive PID). */
        protected boolean alternates() { return true; }

        @Override
        protected void initTuner() {
            buildPaths();
            telemetry.addLine(instructions());
            telemetry.addLine("B = stop. DPad/bumpers edit gains" + (editors.size() > 1 ? ", Y = next controller." : "."));
            telemetry.update();
        }

        @Override
        public void start() {
            follower.followPath(forward);
            setControllers();
        }

        @Override
        public void loop() {
            if (gamepad1.b) {
                follower.breakFollowing();
                stopped = true;
            }
            follower.update();
            if (!stopped && alternates() && !follower.isBusy()) {
                goingForward = !goingForward;
                follower.followPath(goingForward ? forward : backward);
                setControllers();
            }
            if (pressed(0, gamepad1.y) && !editors.isEmpty()) editorIndex = (editorIndex + 1) % editors.size();
            if (!editors.isEmpty()) editors.get(editorIndex).handle(gamepad1);

            Pose p = follower.getPose();
            Vector te = follower.getTranslationalError();
            telemetry.addLine(stopped ? "STOPPED (restart the OpMode to run again)" : instructions());
            telemetry.addData("Pose", "X %.1f  Y %.1f  H %.1f deg", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
            telemetry.addData("Translational err", "%.2f in (x %.2f, y %.2f)", te.getMagnitude(), te.getXComponent(), te.getYComponent());
            telemetry.addData("Heading err", "%.2f deg", Math.toDegrees(follower.getHeadingError()));
            telemetry.addData("Drive err", "%.2f", follower.getDriveError());
            if (!editors.isEmpty()) editors.get(editorIndex).show(telemetry);
            telemetry.update();
        }

        protected void buildLine() {
            forward = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(PATH_DISTANCE, 0, 0)))
                    .setConstantHeadingInterpolation(0)
                    .build();
            backward = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(PATH_DISTANCE, 0, 0), new Pose(0, 0, 0)))
                    .setConstantHeadingInterpolation(0)
                    .build();
        }
    }

    /**
     * Translational PID only. The robot holds the line; PUSH IT SIDEWAYS by hand.
     * Raise P until it snaps back quickly; add D if it oscillates. Keep I = 0 unless it never quite gets back.
     */
    @TeleOp(name = "Translational Tuner", group = GROUP)
    public static class TranslationalTuner extends PathTuner {
        @Override protected void buildPaths() {
            buildLine();
            editors.add(GainEditor.pidf("TRANSLATIONAL", "translationalPIDFCoefficients",
                    Constants.followerConstants.coefficientsTranslationalPIDF));
        }
        @Override protected void setControllers() {
            follower.deactivateAllPIDFs();
            follower.activateTranslational();
        }
        @Override protected boolean alternates() { return false; }
        @Override protected String instructions() { return "Push the robot SIDEWAYS; it should snap back without wobbling."; }
    }

    /**
     * Heading PID only. TURN THE ROBOT by hand; it should turn back to heading 0.
     */
    @TeleOp(name = "Heading Tuner", group = GROUP)
    public static class HeadingTuner extends PathTuner {
        @Override protected void buildPaths() {
            buildLine();
            editors.add(GainEditor.pidf("HEADING", "headingPIDFCoefficients",
                    Constants.followerConstants.coefficientsHeadingPIDF));
        }
        @Override protected void setControllers() {
            follower.deactivateAllPIDFs();
            follower.activateHeading();
        }
        @Override protected boolean alternates() { return false; }
        @Override protected String instructions() { return "Twist the robot by hand; it should turn back to 0 deg without wobbling."; }
    }

    /**
     * Drive PID only. The robot drives PATH_DISTANCE forward and back.
     * Tune so it stops at the end without overshooting or crawling.
     */
    @TeleOp(name = "Drive Tuner", group = GROUP)
    public static class DriveTuner extends PathTuner {
        @Override protected void buildPaths() {
            buildLine();
            editors.add(GainEditor.filtered("DRIVE", "drivePIDFCoefficients",
                    Constants.followerConstants.coefficientsDrivePIDF));
        }
        @Override protected void setControllers() {
            follower.deactivateAllPIDFs();
            follower.activateDrive();
        }
        @Override protected String instructions() { return "Drives forward/back. Should stop at each end without overshoot."; }
    }

    /**
     * Drives a curve forward and back with all controllers on. Raise centripetal scaling
     * if the robot swings to the OUTSIDE of the curve, lower it if it cuts INSIDE.
     */
    @TeleOp(name = "Centripetal Tuner", group = GROUP)
    public static class CentripetalTuner extends PathTuner {
        @SuppressLint("DefaultLocale")
        @Override protected void buildPaths() {
            forward = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(0, 0), new Pose(CURVE_SIZE, 0), new Pose(CURVE_SIZE, CURVE_SIZE)))
                    .setTangentHeadingInterpolation()
                    .build();
            backward = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(CURVE_SIZE, CURVE_SIZE), new Pose(CURVE_SIZE, 0), new Pose(0, 0)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            editors.add(new GainEditor("CENTRIPETAL", new String[]{"scaling"},
                    new GainEditor.Getter[]{() -> Constants.followerConstants.centripetalScaling},
                    new GainEditor.Setter[]{v -> {
                        Constants.followerConstants.centripetalScaling = v;
                        follower.setCentripetalScaling(v);
                    }},
                    () -> String.format(".centripetalScaling(%s)", GainEditor.n(Constants.followerConstants.centripetalScaling))));
        }
        @Override protected void setControllers() {
            follower.activateAllPIDFs();
        }
        @Override protected String instructions() { return "Curve forward-left and back. Swings OUT -> raise; cuts IN -> lower. Use step 0.0001."; }
    }

    // =========================================================================================
    // 4. CHECK
    // =========================================================================================

    /** Everything on. Drives a line back and forth. Y switches which controller you edit. */
    @TeleOp(name = "Line Test", group = GROUP)
    public static class LineTest extends PathTuner {
        @Override protected void buildPaths() {
            buildLine();
            editors.add(GainEditor.pidf("TRANSLATIONAL", "translationalPIDFCoefficients",
                    Constants.followerConstants.coefficientsTranslationalPIDF));
            editors.add(GainEditor.pidf("HEADING", "headingPIDFCoefficients",
                    Constants.followerConstants.coefficientsHeadingPIDF));
            editors.add(GainEditor.filtered("DRIVE", "drivePIDFCoefficients",
                    Constants.followerConstants.coefficientsDrivePIDF));
        }
        @Override protected void setControllers() {
            follower.activateAllPIDFs();
        }
        @Override protected String instructions() { return "Full test: straight, no drift, clean stops at both ends."; }
    }
}
