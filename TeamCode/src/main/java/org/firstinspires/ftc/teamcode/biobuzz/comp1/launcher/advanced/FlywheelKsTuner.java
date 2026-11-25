package org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.advanced;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * STEP 1 of 3 - kS (static friction). Sends a fixed "12 V power" = kS to the flywheel.
 *
 * Raise kS in small steps until the wheel JUST starts turning on its own. Once it is spinning it keeps going at a
 * lower power, so press A to stop it, then come back up from below each time. Do 3-5 tries (cold and warm motor)
 * and use the average. Type it into Flywheel.KS.
 *
 * gamepad1:
 *   dpad up / down    - kS + / - step
 *   dpad right / left - bigger / smaller step
 *   A                 - stop (power 0) until you change kS again
 */
@TeleOp(name = "Flywheel kS Tuner (1 of 3)", group = "BioBuzz Flywheel")
public class FlywheelKsTuner extends OpMode {

    private final Flywheel flywheel = new Flywheel();
    private double kS = Flywheel.KS;
    private final double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    private int incrementIdx = 4; // start at 0.01
    private boolean stopped = false;

    @Override
    public void init() {
        flywheel.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed() && incrementIdx < increments.length - 1) incrementIdx++;
        else if (gamepad1.dpadLeftWasPressed() && incrementIdx > 0) incrementIdx--;
        double step = increments[incrementIdx];

        if (gamepad1.dpadUpWasPressed()) { kS += step; stopped = false; }
        if (gamepad1.dpadDownWasPressed()) { kS = Math.max(0, kS - step); stopped = false; }
        if (gamepad1.aWasPressed()) stopped = true;

        Flywheel.KS = kS; // carries over to the kV / kP tuners until the robot app restarts
        if (stopped) flywheel.stopMotors();
        else flywheel.setCompensatedPower(kS);

        telemetry.addData("Step (dpad L/R)", "%.6f", step);
        telemetry.addData("kS (dpad U/D)", "%.6f   %s", kS, stopped ? "STOPPED (A)" : "");
        telemetry.addData("RPM", "%.0f", flywheel.getRPM());
        telemetry.addData("Ticks per sec", "%.0f", flywheel.getTicksPerSec());
        telemetry.addData("Battery", "%.2f V", flywheel.batteryVoltage());
        telemetry.addLine("Find the kS where it JUST starts turning, then put it in Flywheel.KS");
    }

    @Override
    public void stop() {
        flywheel.stopMotors();
    }
}
