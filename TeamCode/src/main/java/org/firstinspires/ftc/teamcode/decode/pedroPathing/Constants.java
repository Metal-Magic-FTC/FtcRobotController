package org.firstinspires.ftc.teamcode.decode.pedroPathing;

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

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.78)
            .forwardZeroPowerAcceleration(-37.52)
            .lateralZeroPowerAcceleration(-71.17)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.075, 0.0001, 0.0115, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(0.75,0, 0.003, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.8, 0.05, 0.01, 0.6, 0.03));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(70.75)
            .yVelocity(59.43);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.5)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
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