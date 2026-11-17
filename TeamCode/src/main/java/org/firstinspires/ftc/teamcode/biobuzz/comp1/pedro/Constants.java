package org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3 constants. Fill these in with AutoTune (see Tuning.java): open http://192.168.43.1:10158 while
 * connected to the robot, run each procedure, and paste the code from its Java tab over the matching config below.
 */
public class Constants {

    // Robot dimensions, inches. Pedro does not use these (it learns the drivetrain from tuning);
    // they are here for start poses / geometry.
    public static final double ROBOT_WIDTH_IN = 10.5;
    public static final double ROBOT_LENGTH_IN = 16.5;
    /** Outside of wheel to outside of wheel. */
    public static final double WHEEL_SPAN_WIDTH_IN = 16.0;
    public static final double WHEEL_SPAN_LENGTH_IN = 15.5;
    /** goBILDA mecanum. */
    public static final double WHEEL_DIAMETER_MM = 104.0;

    /** Mecanum procedure. Directions are carried over from Pedro 2 and are NOT verified - run the tuner. */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    /**
     * Pinpoint procedure. Values from the measured pod positions and the Pinpoint Direction Check:
     * x pod = forward pod (3 in right of center), y pod = strafe pod (level with center), both reversed.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.7679692065621926);
        c.yPodOffset.set(-6.064584687000185);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /**
     * Foresight procedure. Every value here comes from AutoTune; until it is pasted in, create() throws
     * "Config variable has not been set", so nothing can drive on untuned gains.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.38486741992146295);
                Controller secondaryTranslationalForward = Controller.proportional(0.14219822781411995);
                Controller primaryTranslationalLateral = Controller.proportional(0.6948331376680421);
                Controller secondaryTranslationalLateral = Controller.proportional(0.25672227808496295);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013879707962918996));
                c.brake.set(Controller.proportionalFeedforward(0.011797751768481147));

                c.headingFeedback.set(Controller.proportional(5.9980885064244));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05905882214601103, 0.0050167274903517));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07853019311996481, 0.0834335140395966));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00200796369403241, 0.0014041418810816057));

                c.maxAchievableForwardVelocity.set(72.7233654593414);
                c.maxAchievableStrafeVelocity.set(57.458474701657835);
                c.naturalForwardDeceleration.set(36.089705838117524);
                c.naturalStrafeDeceleration.set(69.89505877231639);
            }
    );

    public static Follower create(HardwareMap h) {
        // Bulk reads: all motor encoder/velocity reads in a loop share one hub transaction -> faster, steadier loops.
        for (LynxModule hub : h.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
