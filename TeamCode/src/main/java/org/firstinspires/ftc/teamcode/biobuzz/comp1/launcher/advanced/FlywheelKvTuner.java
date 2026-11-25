package org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.advanced;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * STEP 2 of 3 - kV (velocity feedforward). Sends "12 V power" = kV * goalRPM + kS, no feedback.
 * Set Flywheel.KS from step 1 first.
 *
 * Adjust kV until the wheel settles right at (or just under) the goal RPM. Then press B to check it roughly holds
 * the LOW goal too (it won't be perfect - kP fixes the rest). Type it into Flywheel.KV.
 *
 * gamepad1:
 *   dpad up / down    - kV + / - step
 *   dpad right / left - bigger / smaller step
 *   A / B             - goal = HIGH / LOW RPM
 *   X                 - flywheel on / off (starts ON)
 */
@TeleOp(name = "Flywheel kV Tuner (2 of 3)", group = "BioBuzz Flywheel")
public class FlywheelKvTuner extends OpMode {

    /** HIGH = the old wall shot (1650 ticks/s); LOW = a slower target to check kV scales. Flywheel RPM. */
    public static final double HIGH_RPM = Flywheel.ticksPerSecToRpm(1650);
    public static final double LOW_RPM = Flywheel.ticksPerSecToRpm(1000);

    private final Flywheel flywheel = new Flywheel();
    private double kV = Flywheel.KV;
    private double goalRpm = HIGH_RPM;
    private boolean on = true;
    private final double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    private int incrementIdx = 1; // start at 0.00001

    @Override
    public void init() {
        flywheel.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed() && incrementIdx < increments.length - 1) incrementIdx++;
        else if (gamepad1.dpadLeftWasPressed() && incrementIdx > 0) incrementIdx--;
        if (gamepad1.aWasPressed()) goalRpm = HIGH_RPM;
        if (gamepad1.bWasPressed()) goalRpm = LOW_RPM;
        if (gamepad1.xWasPressed()) on = !on;
        double step = increments[incrementIdx];

        if (gamepad1.dpadUpWasPressed()) kV += step;
        if (gamepad1.dpadDownWasPressed()) kV = Math.max(0, kV - step);

        Flywheel.KV = kV; // carries over to the kP tuner until the robot app restarts
        double power = kV * goalRpm + Flywheel.KS;
        if (on) flywheel.setCompensatedPower(power);
        else flywheel.stopMotors();

        double rpm = flywheel.getRPM();
        telemetry.addData("Step (dpad L/R)", "%.6f", step);
        telemetry.addData("kV (dpad U/D)", "%.7f", kV);
        telemetry.addData("kS (from Flywheel.KS)", "%.4f", Flywheel.KS);
        telemetry.addData("Goal (A high / B low)", "%.0f RPM", goalRpm);
        telemetry.addData("RPM", "%.0f   (error %+.0f)", rpm, goalRpm - rpm);
        telemetry.addData("Ticks per sec", "%.0f", flywheel.getTicksPerSec());
        telemetry.addData("Power (12 V)", "%.3f   battery %.2f V", power, flywheel.batteryVoltage());
        telemetry.addData("Flywheel (X)", on ? "ON" : "OFF");
        telemetry.addLine("Get RPM to sit at / just under the HIGH goal, then put kV in Flywheel.KV");
    }

    @Override
    public void stop() {
        flywheel.stopMotors();
    }
}
