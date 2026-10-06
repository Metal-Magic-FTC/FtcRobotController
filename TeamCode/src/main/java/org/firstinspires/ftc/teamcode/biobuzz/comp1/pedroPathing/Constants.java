package org.firstinspires.ftc.teamcode.biobuzz.comp1.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // Robot dimensions, inches. Pedro itself does not use these (it learns the drivetrain from tuning);
    // they are here for start poses / geometry.
    public static final double ROBOT_WIDTH_IN = 17.5;
    public static final double ROBOT_LENGTH_IN = 16.0;
    /** Outside of wheel to outside of wheel. */
    public static final double WHEEL_SPAN_WIDTH_IN = 16.0;
    public static final double WHEEL_SPAN_LENGTH_IN = 15.5;
    /** goBILDA mecanum. */
    public static final double WHEEL_DIAMETER_MM = 104.0;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.8);
//            .forwardZeroPowerAcceleration(66.87291134811763
//          .lateralZeroPowerAcceleration(-71.17);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true);
            //.xVelocity(70.75)
            //.yVelocity(59.43);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3)   // forward pod is 3 in RIGHT of robot center (left = +)
            .strafePodX(0)     // strafe pod is level with robot center front-to-back (forward = +)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);

    public static DriveEncoderConstants driveEncoderConstants = new DriveEncoderConstants()
            .robotWidth(18)
            .robotLength(18)
            .forwardTicksToInches(-10000)
            .strafeTicksToInches(-11000)
            .turnTicksToInches(0.95);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)

                .build();
    }
}