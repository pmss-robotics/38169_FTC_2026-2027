package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * KICKER: one standard (positional) servo that pushes balls into the flywheel.
 *
 * While shooting, it flips back and forth between KICK and REST as fast as the servo can
 * physically travel, so each kick loads one ball. When not shooting it sits at REST.
 *
 * Configure Robot name: "kicker".
 *
 * TUNING (do this with the flywheel OFF):
 *   1. Set KICK_TIME and REST_TIME large (0.5) so you can watch each move.
 *   2. Adjust REST_POSITION / KICK_POSITION until the arm stops in the right spots.
 *   3. Lower the two times until the servo just barely reaches each position before
 *      turning around. Too short = it never finishes a full kick.
 */
public class KickerSubsystem {

    // ---- TUNE THESE ----
    public static final double REST_POSITION = 0.0;  // placeholder, 0..1
    public static final double KICK_POSITION = 0.5;  // placeholder, 0..1
    public static final double KICK_TIME = 0.15;     // seconds to reach KICK from REST
    public static final double REST_TIME = 0.15;     // seconds to reach REST from KICK

    private final Servo servo;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean kicking = false;   // true = currently at/moving to KICK_POSITION
    private boolean active = false;    // true = cycling (shoot button held)

    public KickerSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "kicker");
        servo.setDirection(Servo.Direction.FORWARD);
        rest();
    }

    /**
     * Call once every loop.
     * @param shoot true while the shoot button is held: kicker cycles KICK/REST nonstop.
     *              false: kicker goes to REST and waits.
     */
    public void update(boolean shoot) {
        if (!shoot) {
            if (active) rest();
            return;
        }

        if (!active) {
            // Just started shooting: kick right away.
            active = true;
            goTo(true);
            return;
        }

        double waitTime = kicking ? KICK_TIME : REST_TIME;
        if (timer.seconds() >= waitTime) {
            goTo(!kicking);
        }
    }

    /** Go to REST and stop cycling. */
    public void rest() {
        active = false;
        goTo(false);
    }

    private void goTo(boolean kick) {
        kicking = kick;
        servo.setPosition(kick ? KICK_POSITION : REST_POSITION);
        timer.reset();
    }

    public boolean isActive() { return active; }
    public boolean isKicking() { return kicking; }
    public double getPosition() { return servo.getPosition(); }
}
