package org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.advanced;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.LauncherConstants;

/**
 * Custom flywheel controller (kS + kV feedforward + kP feedback), following "the definitive course on flywheel
 * tuning" video, plus two additions for multi-ball shooting:
 *
 *   power = kS + kV * targetRPM + kP * (targetRPM - RPM)          (video)
 *   x 12V / battery voltage  -> same speed on a full or tired battery (VOLTAGE COMPENSATION)
 *   full power while the wheel is more than BOOST_BELOW_FRACTION under target -> fast recovery after each ball
 *                                                                                  (DIP BOOST)
 *
 * The motor runs RUN_WITHOUT_ENCODER (we do the control, not the hub) but the encoder is still read.
 * RPM here is FLYWHEEL (wheel) RPM. kS / kV / kP are "12 V power": tune them with the tuners in this package
 * (FlywheelKsTuner -> FlywheelKvTuner -> FlywheelKpTuner), then type the numbers into KS / KV / KP below.
 *
 * Call setMotorRPM() EVERY loop (it is a custom controller - it only corrects when called).
 */
public class Flywheel {

    // ======================= TUNED VALUES (12 V normalized) =======================
    /** Power that just barely gets the wheel turning. FlywheelKsTuner. */
    public static double KS = 0.11; // measured with FlywheelKsTuner
    /** Power per flywheel RPM that holds a target without P. FlywheelKvTuner. ~1 / free RPM to start. */
    public static double KV = 0.000168; // measured with FlywheelKvTuner
    /** Extra power per RPM of error. FlywheelKpTuner. */
    public static double KP = 0.00162; // BioBuzzTeleOp gamepad1 dpad down cycles 0.001 / 0.0012 / 0.0015 / 0.00162 / 0.0018

    // ======================= DIP BOOST =======================
    public static boolean BOOST_ENABLED = true;
    /** Go to full power when the wheel falls this far under the target (a ball just went through)... */
    public static double BOOST_BELOW_FRACTION = 0.05;
    /** ...and hand back to kS/kV/kP once it is back within this. */
    public static double BOOST_EXIT_FRACTION = 0.015;

    // ======================= HARDWARE =======================
    public static final double ENCODER_CPR = 28.0; // goBILDA / REV bare motor encoder, counts per motor rev
    /** Motor revs per flywheel rev (1 = direct drive). From LauncherConstants. */
    public static final double GEAR_RATIO = LauncherConstants.MOTOR_REVS_PER_WHEEL_REV;
    public static final double NOMINAL_VOLTAGE = 12.0;

    private DcMotorEx motor;
    private VoltageSensor voltageSensor;
    private double voltage = NOMINAL_VOLTAGE;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private boolean boosting = false;
    private double lastPower = 0;

    public void init(HardwareMap hardwareMap) {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        motor = hardwareMap.get(DcMotorEx.class, LauncherConstants.MOTOR_NAME);
        motor.setDirection(LauncherConstants.MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down, don't brake the wheel
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we control it; the encoder is still read
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
    }

    // ======================= RAW =======================
    /** Raw motor power, -1..1 (no compensation). */
    public void setMotorPower(double power) {
        lastPower = Range.clip(power, -1, 1);
        motor.setPower(lastPower);
    }

    /** "12 V power": scaled up as the battery drops so the wheel behaves the same. Used by the tuners. */
    public void setCompensatedPower(double power12v) {
        setMotorPower(power12v * NOMINAL_VOLTAGE / batteryVoltage());
    }

    public void stopMotors() {
        boosting = false;
        setMotorPower(0);
    }

    // ======================= CONTROLLER =======================
    /** The custom controller. Call EVERY loop while the flywheel should hold targetRpm. */
    public void setMotorRPM(double targetRpm) {
        if (targetRpm <= 0) {
            stopMotors();
            return;
        }
        double rpm = getRPM();
        double error = targetRpm - rpm;

        if (BOOST_ENABLED) {
            if (!boosting && rpm < targetRpm * (1 - BOOST_BELOW_FRACTION)) boosting = true;
            if (boosting && rpm >= targetRpm * (1 - BOOST_EXIT_FRACTION)) boosting = false;
        } else {
            boosting = false;
        }

        if (boosting) {
            setMotorPower(1.0);
            return;
        }
        double feedForward = KV * targetRpm + KS;
        double feedBack = KP * error;
        setCompensatedPower(feedForward + feedBack);
    }

    // ======================= STATE =======================
    public double getTicksPerSec() {
        return motor.getVelocity();
    }

    /** Flywheel RPM (after the gear ratio). */
    public double getRPM() {
        return getTicksPerSec() / ENCODER_CPR * 60.0 / GEAR_RATIO;
    }

    /** Ticks/s at the motor encoder for a flywheel RPM (to compare with the old ticks/s speeds). */
    public static double rpmToTicksPerSec(double rpm) {
        return rpm * GEAR_RATIO * ENCODER_CPR / 60.0;
    }

    public static double ticksPerSecToRpm(double tps) {
        return tps / ENCODER_CPR * 60.0 / GEAR_RATIO;
    }

    /** Battery voltage, re-read at most every 100 ms (each read costs loop time). */
    public double batteryVoltage() {
        if (voltageTimer.milliseconds() > 100) {
            double v = voltageSensor.getVoltage();
            if (v > 6) voltage = v; // ignore a bad read
            voltageTimer.reset();
        }
        return voltage;
    }

    public boolean isBoosting() {
        return boosting;
    }

    public double lastPower() {
        return lastPower;
    }
}
