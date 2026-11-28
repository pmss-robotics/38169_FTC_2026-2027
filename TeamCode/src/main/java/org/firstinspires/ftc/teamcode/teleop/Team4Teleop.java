package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeChloe;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Team4Drive;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.XAxisMotor;
import org.firstinspires.ftc.teamcode.subsystems.YAxisMotor;

/*
 * TEAM 4 COMPETITION TELEOP (everything on gamepad 1)
 *
 *   Left stick         : field-centric drive (move)
 *   Right stick X      : turn (heading)
 *   Left trigger (hold): shoot, kicker cycles KICK/REST to load balls into the flywheel
 *   Y (tap)            : toggle intake + transfer
 *   X (tap)            : toggle shooter flywheel
 *   Back (hold)        : reverse intake + transfer + flywheel to clear jams
 *   D-pad up / down    : raise / lower shooter ramp
 *   D-pad left / right : turn turret left / right
 *   Start (tap)        : re-zero heading (point robot straight away from the drivers first)
 *
 * FUTURE (not wired yet):
 *   B                  : blocker toggle
 *   VisionAim toggle   : auto-aim turret with the Limelight, disables D-pad left/right
 */
@TeleOp(name = "Team 4 Teleop", group = "Teleop")
public class Team4Teleop extends LinearOpMode {

    // ---- TUNE THESE ----
    private static final int INPUT_CURVE = 3;              // 1 linear, 2 quadratic, 3 cubic
    private static final double SHOOT_RPM = 4500.0;        // flywheel speed when X is on
    private static final double AT_SPEED_TOLERANCE_RPM = 150.0;
    private static final double REVERSE_RPM = 1500.0;      // flywheel speed while Back is held
    private static final double TRIGGER_THRESHOLD = 0.5;   // how far to pull LT to shoot
    private static final double RAMP_DEG_PER_SEC = 30.0;   // ramp speed while D-pad held
    private static final double TURRET_POWER = 0.4;        // turret power while D-pad held

    // What a group of mechanisms is currently doing (only send hardware commands on change)
    private enum Mode { OFF, FORWARD, REVERSE }

    private Team4Drive drive;
    private IntakeChloe intake;
    private TransferSubsystem transfer;
    private FlyWheelSubsystem flywheel;
    private KickerSubsystem kicker;
    private XAxisMotor turret;
    private YAxisMotor ramp;

    @Override
    public void runOpMode() {
        // FTCLib subsystems register themselves with the scheduler. Clear any left over
        // from a previous OpMode so nothing stale ever runs.
        CommandScheduler.getInstance().reset();

        drive = new Team4Drive(hardwareMap);
        intake = new IntakeChloe();
        intake.init(hardwareMap);
        transfer = new TransferSubsystem();
        transfer.init(hardwareMap);
        flywheel = new FlyWheelSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        turret = new XAxisMotor(hardwareMap, telemetry);  // zeroes turret encoder: start CENTERED
        ramp = new YAxisMotor(hardwareMap, telemetry);    // zeroes ramp encoder: start fully DOWN

        // Toggle states
        boolean intakeOn = false;
        boolean flywheelOn = false;

        // Last loop's button states, so one press only counts once
        boolean lastY = false, lastX = false, lastStart = false;

        // What we last told the hardware, so we only send commands when something changes
        Mode intakeMode = Mode.OFF;
        Mode flywheelMode = Mode.OFF;
        boolean turretMoving = false;

        telemetry.addLine("Team 4 Teleop ready");
        telemetry.addLine("Turret CENTERED, ramp DOWN, robot facing away from drivers");
        telemetry.addData("IMU angles", drive.getAnglesDebug());
        telemetry.update();

        waitForStart();
        drive.resetHeading();
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // ---------------- DRIVE ----------------
            if (gamepad1.start && !lastStart) {
                drive.resetHeading();
            }
            drive.drive(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    INPUT_CURVE
            );

            // ---------------- TOGGLES ----------------
            if (gamepad1.y && !lastY) intakeOn = !intakeOn;
            if (gamepad1.x && !lastX) flywheelOn = !flywheelOn;

            // ---------------- INTAKE / TRANSFER / FLYWHEEL ----------------
            // Back overrides everything while held; toggles resume when it's released.
            boolean clearing = gamepad1.back;

            Mode wantIntake = clearing ? Mode.REVERSE : (intakeOn ? Mode.FORWARD : Mode.OFF);
            if (wantIntake != intakeMode) {
                intakeMode = wantIntake;
                switch (intakeMode) {
                    case FORWARD: intake.in();  transfer.run();     break;
                    case REVERSE: intake.out(); transfer.reverse(); break;
                    default:      intake.stop(); transfer.stop();   break;
                }
            }

            Mode wantFlywheel = clearing ? Mode.REVERSE : (flywheelOn ? Mode.FORWARD : Mode.OFF);
            if (wantFlywheel != flywheelMode) {
                flywheelMode = wantFlywheel;
                switch (flywheelMode) {
                    case FORWARD: flywheel.setVelocityRPM(SHOOT_RPM); break;
                    case REVERSE: flywheel.reverse(REVERSE_RPM);      break;
                    default:      flywheel.stop();                    break;
                }
            }

            // ---------------- SHOOT (kicker) ----------------
            boolean shooting = !clearing && gamepad1.left_trigger > TRIGGER_THRESHOLD;
            kicker.update(shooting);

            // ---------------- RAMP (D-pad up/down) ----------------
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                ramp.adjustAngle(RAMP_DEG_PER_SEC * dt);
            } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                ramp.adjustAngle(-RAMP_DEG_PER_SEC * dt);
            }

            // ---------------- TURRET (D-pad left/right) ----------------
            // TODO VisionAim: when VisionAim is on, skip this block and aim from the Limelight.
            // If left/right is backwards, swap true/false below (or flip the motor direction).
            if (gamepad1.dpad_right && !gamepad1.dpad_left) {
                turret.movement(TURRET_POWER, true);
                turretMoving = true;
            } else if (gamepad1.dpad_left && !gamepad1.dpad_right) {
                turret.movement(TURRET_POWER, false);
                turretMoving = true;
            } else if (turretMoving) {
                turret.stop();
                turretMoving = false;
            }

            // TODO Blocker: B toggles blocker.deploy() / blocker.retract() once BlockerMatthew is fixed.

            lastY = gamepad1.y;
            lastX = gamepad1.x;
            lastStart = gamepad1.start;

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Heading (deg)", "%.1f", drive.getHeadingDegrees());
            telemetry.addData("Intake/Transfer", clearing ? "REVERSING" : (intakeOn ? "ON" : "off"));
            telemetry.addData("Flywheel", clearing ? "REVERSING"
                    : (!flywheelOn ? "off"
                    : (flywheel.isAtSpeed(AT_SPEED_TOLERANCE_RPM) ? "READY" : "spinning up")));
            telemetry.addData("Kicker", shooting ? "SHOOTING" : "rest");
            flywheel.periodic();  // target / actual RPM
            ramp.periodic();      // ramp target / current angle
            turret.periodic();    // turret ticks
            telemetry.update();
        }

        // Make sure nothing keeps running after STOP, and leave the scheduler clean
        intake.stop();
        transfer.stop();
        flywheel.stop();
        turret.stop();
        CommandScheduler.getInstance().reset();
    }
}
