package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/*
 * BLOCKER, starter code: two DC motors that turn in OPPOSITE directions.
 *
 * Why opposite? The two motors are mounted facing each other (mirrored), so to move the
 * blocker the same way, one motor has to spin clockwise while the other spins counter-
 * clockwise. We handle that ONCE here with setDirection(REVERSE) on one motor. After that,
 * the rest of the code treats both motors the same: one target position, one power.
 *
 * It moves between two positions (retracted / deployed), measured in encoder ticks, and
 * holds there with the motors' built-in position control.
 *
 * Configure Robot names: "blockerLeft" and "blockerRight" (change below if yours differ).
 *
 * FIRST RUN (do this before anything else):
 *   1. Keep the blocker clear of anything it can hit, and use LOW_POWER first.
 *   2. Make sure retracted is the starting position, then run the OpMode so ticks start at 0.
 *   3. If the blocker moves the WRONG way or the two motors fight each other (loud, hot,
 *      stalls), swap which motor has REVERSE in the constructor. They must push together.
 *   4. Use the D-pad to nudge to the real deployed spot, read the ticks on screen, and write
 *      that number into DEPLOYED_TICKS.
 */
public class BlockerMatthew {

    // ---- TUNE THESE ----
    public static final int RETRACTED_TICKS = 0;
    public static final int DEPLOYED_TICKS = 300;   // placeholder: find your real number
    public static final int MIN_TICKS = -20;        // never go past these (protects the frame)
    public static final int MAX_TICKS = 400;        // placeholder: a bit past DEPLOYED_TICKS
    public static final int NUDGE_TICKS = 10;       // size of one manual nudge
    public static final int TOLERANCE_TICKS = 8;    // "close enough" to count as arrived
    public static final double LOW_POWER = 0.3;     // use this for first tests
    public static final double POWER = 0.6;         // normal moving power, 0 to 1

    private final DcMotorEx left;
    private final DcMotorEx right;
    private int target = RETRACTED_TICKS;
    private double power = POWER;

    public BlockerMatthew(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "blockerLeft");
        right = hardwareMap.get(DcMotorEx.class, "blockerRight");

        // Opposite directions: this is the whole trick. If it's backwards, swap these two.
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        // BRAKE makes the motors resist movement when they are not being driven.
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
        setTarget(RETRACTED_TICKS);
    }

    /** Treat the current position as 0 ticks. Do this with the blocker fully retracted. */
    public void resetEncoders() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        target = 0;
        setTarget(0);
    }

    /** Move to a target in ticks (clamped to the safe range). Both motors get the same target. */
    public void setTarget(int ticks) {
        target = Range.clip(ticks, MIN_TICKS, MAX_TICKS);
        left.setTargetPosition(target);
        right.setTargetPosition(target);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(power);
        right.setPower(power);
    }

    public void deploy() { setTarget(DEPLOYED_TICKS); }

    public void retract() { setTarget(RETRACTED_TICKS); }

    /** Small manual move: positive = toward deployed, negative = toward retracted. */
    public void nudge(int direction) { setTarget(target + direction * NUDGE_TICKS); }

    /** Use LOW_POWER while testing and POWER once it works. */
    public void setPower(double newPower) {
        power = Range.clip(Math.abs(newPower), 0, 1);
        left.setPower(power);
        right.setPower(power);
    }

    /** True when both motors are within tolerance of the target. */
    public boolean atTarget() {
        return Math.abs(left.getCurrentPosition() - target) <= TOLERANCE_TICKS
                && Math.abs(right.getCurrentPosition() - target) <= TOLERANCE_TICKS;
    }

    /** Cut power right now (for emergencies). */
    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    public int getTarget() { return target; }
    public int getLeftPosition() { return left.getCurrentPosition(); }
    public int getRightPosition() { return right.getCurrentPosition(); }

    /** True if either motor is drawing too much current (the blocker is probably jammed). */
    public boolean isJammed() { return left.isOverCurrent() || right.isOverCurrent(); }
}