package org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.procedures.Tests;

/**
 * AutoTune procedures. Each @Tuner method shows up as a procedure on the AutoTune page
 * (http://192.168.43.1:10158 while connected to the robot's Wi-Fi). Run them in this order:
 *   1. Mecanum   -> paste result into Constants.drivetrainConfig
 *   2. Pinpoint  -> paste result into Constants.localizerConfig
 *   3. Foresight -> paste result into Constants.foresightConfig
 *   4. Tests     -> Localization / Line / Curve / Hold checks
 * Reinstall the code after each paste so the next procedure uses the new values.
 */
public class Tuning {

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
