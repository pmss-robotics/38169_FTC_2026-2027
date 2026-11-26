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

        public static FollowerConstants followerConstants = new FollowerConstants()
                // TODO: weigh the robot (with battery) and enter kg
                .mass(5)
                .forwardZeroPowerAcceleration(-74)
                .lateralZeroPowerAcceleration(-87)
                .translationalPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.1, 0, 0., 0))
                .drivePIDFCoefficients(new com.pedropathing.control.FilteredPIDFCoefficients(0.025, 0.0, 1.0E-5, 0.6, 0.01))
                .centripetalScaling(5.0E-4)
                .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(1.0, 0, 0,0.01));


        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

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

                .xVelocity(78.8)
                .yVelocity(71.3)
                .useBrakeModeInTeleOp(true);

        public static PinpointConstants localizerConstants = new PinpointConstants()
                //TODO: Measure the distance to each x, y localization sensor
                .forwardPodY(-3.5) // placeholder
                .strafePodX(-6.5)  // placeholder

                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

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