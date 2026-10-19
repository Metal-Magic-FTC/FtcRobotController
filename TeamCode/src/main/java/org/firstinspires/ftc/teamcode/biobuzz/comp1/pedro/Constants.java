package org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3 constants. Fill these in with AutoTune (see Tuning.java): open http://192.168.43.1:10158 while
 * connected to the robot, run each procedure, and paste the code from its Java tab over the matching config below.
 */
public class Constants {

    // Robot dimensions, inches. Pedro does not use these (it learns the drivetrain from tuning);
    // they are here for start poses / geometry.
    public static final double ROBOT_WIDTH_IN = 17.5;
    public static final double ROBOT_LENGTH_IN = 16.0;
    /** Outside of wheel to outside of wheel. */
    public static final double WHEEL_SPAN_WIDTH_IN = 16.0;
    public static final double WHEEL_SPAN_LENGTH_IN = 15.5;
    /** goBILDA mecanum. */
    public static final double WHEEL_DIAMETER_MM = 104.0;

    /** Mecanum procedure. Directions are carried over from Pedro 2 and are NOT verified - run the tuner. */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.backLeftName.set("backLeft");
        c.frontRightName.set("frontRight");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    /**
     * Pinpoint procedure. Values from the measured pod positions and the Pinpoint Direction Check:
     * x pod = forward pod (3 in right of center), y pod = strafe pod (level with center), both reversed.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /**
     * Foresight procedure. Every value here comes from AutoTune; until it is pasted in, create() throws
     * "Config variable has not been set", so nothing can drive on untuned gains.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Paste the ForesightConfig body from the AutoTune Java tab here.
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
