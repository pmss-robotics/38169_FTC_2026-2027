package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Spins each drive motor alone at the same power and reports speed + current.
 * Put the robot on blocks (wheels off the ground).
 *
 * Healthy robot: all four motors show about the same ticks/sec and similar amps.
 * A motor that is slower, or pulls much more current, has a mechanical problem
 * (tight chain/belt, bad bearing, worn gearbox, rubbing wheel).
 *
 * Hold A=fl, B=fr, X=bl, Y=br.
 */
@TeleOp(name = "Motor Balance Test", group = "Pedro")
public class MotorBalanceTest extends LinearOpMode {

    private static final double TEST_POWER = 0.5;

    @Override
    public void runOpMode() {
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "fl");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "bl");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "fr");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "br");

        // Same directions as Constants
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx[] motors = {fl, bl, fr, br};
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Wheels off the ground! Hold A=fl  B=fr  X=bl  Y=br");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            fl.setPower(gamepad1.a ? TEST_POWER : 0);
            fr.setPower(gamepad1.b ? TEST_POWER : 0);
            bl.setPower(gamepad1.x ? TEST_POWER : 0);
            br.setPower(gamepad1.y ? TEST_POWER : 0);

            telemetry.addData("fl (A)", "vel %.0f  amps %.2f", fl.getVelocity(), fl.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("fr (B)", "vel %.0f  amps %.2f", fr.getVelocity(), fr.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("bl (X)", "vel %.0f  amps %.2f", bl.getVelocity(), bl.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("br (Y)", "vel %.0f  amps %.2f", br.getVelocity(), br.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("Wheel should spin FORWARD when robot-forward. Compare vel/amps.");
            telemetry.update();
        }
    }
}