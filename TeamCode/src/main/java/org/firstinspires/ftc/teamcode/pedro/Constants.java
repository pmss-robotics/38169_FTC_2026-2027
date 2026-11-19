package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // ---------------------------------------------------------------
    // FOLLOWER (tune these with Pedro's tuning OpModes)
    // ---------------------------------------------------------------
    public static FollowerConstants followerConstants = new FollowerConstants()
            // TODO: weigh the robot (with battery) and enter kg
            .mass(5)
            // TODO: Run "Forward Zero Power Acceleration Tuner" and paste result (negative number)
            .forwardZeroPowerAcceleration(-25.9)
            // TODO: Run "Lateral Zero Power Acceleration Tuner" and paste result (negative number)
            .lateralZeroPowerAcceleration(-67.3);

    // Once the PID tuners are done, add them here, e.g.:
    //   .translationalPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.1, 0, 0.01, 0))
    //   .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(1, 0, 0.01, 0))

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // ---------------------------------------------------------------
    // DRIVETRAIN
    // ---------------------------------------------------------------
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("fl")
            .leftRearMotorName("bl")
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // TODO: Run "Forward Velocity Tuner" and paste result (in/sec)
            .xVelocity(81.3)
            // TODO: Run "Lateral Velocity Tuner" and paste result (in/sec)
            .yVelocity(65.4)
            .useBrakeModeInTeleOp(true);

    // ---------------------------------------------------------------
    // LOCALIZER (you confirmed this works)
    // ---------------------------------------------------------------
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.5)
            .strafePodX(-6.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static Follower create(HardwareMap hardwareMap) {
        return createFollower(hardwareMap);
    }
}