package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Checks whether the Axon's built-in encoder (the 4th wire) tracks the turret while the Axon is in normal SERVO mode.
 *
 * SETUP
 *   - Axon flashed with the Servo Mode firmware (Axon Programmer), range set as wide as it goes.
 *   - Robot config: "turretServo" = Servo (NOT Continuous Rotation Servo), "turretEncoder" = Analog Input (4th wire).
 *
 * HOW TO USE
 *   1. INIT. Nothing moves. The first command starts from where the encoder says the servo already is, so it
 *      should not jump (if it does jump, the encoder is not reading position - that is your answer).
 *   2. Hold the stick to creep to one end, then press LEFT BUMPER to save that voltage as "position 0 end";
 *      creep to the other end and press RIGHT BUMPER for "position 1 end". (A / B jump straight to 0 / 1 instead -
 *      only use them once you know the turret and cables can take the full travel.)
 *   3. Now move around. "Tracking error" compares where the servo was told to go with where the encoder says it is.
 *      Small and steady when stopped = the encoder tracks in servo mode.
 *   4. Press Y for an automatic slow sweep end to end; it reports the worst error it saw.
 *
 * gamepad1:
 *   left stick X      - creep the commanded position (slow)
 *   dpad left / right - step the position by 0.01
 *   A / B / X         - jump to 0 / 1 / 0.5   (BIG MOVE - watch the cables)
 *   LB / RB           - save the current voltage as the 0 end / 1 end
 *   Y                 - start / cancel the automatic sweep
 */
@TeleOp(name = "Turret Servo Mode Test (Axon encoder)", group = "BioBuzz Tests")
public class TurretServoModeTest extends OpMode {

    private static final String SERVO_NAME = "turretServo";
    private static final String ENCODER_NAME = "turretEncoder";

    private static final double ENCODER_MAX_VOLTS = 3.3;
    /** Stick creep speed, servo position units per second (0.15 = full travel in about 7 s). */
    private static final double CREEP_PER_SEC = 0.15;
    /** Automatic sweep speed, position units per second. */
    private static final double SWEEP_PER_SEC = 0.10;
    /** Turret turns per servo turn: 3.2x overdrive / 2.75:1 turret ring. */
    private static final double TURRET_REVS_PER_SERVO_REV = 3.2 / 2.75;

    private Servo servo;
    private AnalogInput encoder;

    private double commanded;
    private boolean commandSent = false;
    private double voltsAt0 = 0.0, voltsAt1 = ENCODER_MAX_VOLTS;
    private boolean saved0 = false, saved1 = false;

    private final ElapsedTime loopClock = new ElapsedTime();
    private final ElapsedTime stillClock = new ElapsedTime();

    private boolean sweeping = false;
    private int sweepDir = 1;
    private double sweepWorstError = 0;
    private String sweepResult = "not run yet";

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
        encoder = hardwareMap.get(AnalogInput.class, ENCODER_NAME);
        // Start the command where the encoder says the servo already is, so the first move is not a jump.
        commanded = Range.clip(encoder.getVoltage() / ENCODER_MAX_VOLTS, 0, 1);
        telemetry.addLine("Servo Mode Test ready. Nothing moves until you give a command.");
        telemetry.addData("Encoder now", "%.3f V", encoder.getVoltage());
    }

    @Override
    public void loop() {
        double dt = Math.min(loopClock.seconds(), 0.1);
        loopClock.reset();
        double before = commanded;

        if (gamepad1.yWasPressed()) {
            sweeping = !sweeping;
            if (sweeping) {
                sweepDir = commanded < 0.5 ? 1 : -1;
                sweepWorstError = 0;
                sweepResult = "running...";
            } else {
                sweepResult = "cancelled";
            }
        }

        if (sweeping) {
            commanded += sweepDir * SWEEP_PER_SEC * dt;
            if (commanded >= 1 || commanded <= 0) {
                commanded = Range.clip(commanded, 0, 1);
                sweeping = false;
                sweepResult = String.format("worst error %.3f of travel (%.1f° at the turret)",
                        sweepWorstError, sweepWorstError * turretTravelDeg());
            }
        } else {
            double stick = gamepad1.left_stick_x;
            if (Math.abs(stick) > 0.05) commanded += stick * CREEP_PER_SEC * dt;
            if (gamepad1.dpadRightWasPressed()) commanded += 0.01;
            if (gamepad1.dpadLeftWasPressed()) commanded -= 0.01;
            if (gamepad1.aWasPressed()) commanded = 0;
            if (gamepad1.bWasPressed()) commanded = 1;
            if (gamepad1.xWasPressed()) commanded = 0.5;
        }
        commanded = Range.clip(commanded, 0, 1);

        if (commanded != before) {
            commandSent = true;
            stillClock.reset();
        }
        if (commandSent) servo.setPosition(commanded);

        double volts = encoder.getVoltage();
        if (gamepad1.leftBumperWasPressed()) { voltsAt0 = volts; saved0 = true; }
        if (gamepad1.rightBumperWasPressed()) { voltsAt1 = volts; saved1 = true; }

        double span = voltsAt1 - voltsAt0;
        double measured = Math.abs(span) > 0.05 ? (volts - voltsAt0) / span : Double.NaN;
        double error = measured - commanded;
        if (sweeping && !Double.isNaN(error)) sweepWorstError = Math.max(sweepWorstError, Math.abs(error));

        telemetry.addData("Commanded position", "%.3f%s", commanded, commandSent ? "" : "  (not sent yet)");
        telemetry.addData("Encoder", "%.3f V", volts);
        telemetry.addData("Ends", "0 end %.3f V %s | 1 end %.3f V %s",
                voltsAt0, saved0 ? "(saved)" : "(guess)", voltsAt1, saved1 ? "(saved)" : "(guess)");
        if (Double.isNaN(measured)) {
            telemetry.addLine("Save both ends (LB at one end, RB at the other) to see tracking error.");
        } else {
            telemetry.addData("Encoder position", "%.3f", measured);
            telemetry.addData("Tracking error", "%+.3f of travel = %+.1f° at the turret  (%s)",
                    error, error * turretTravelDeg(),
                    stillClock.seconds() > 1.0 ? "settled" : "moving - lag is normal");
        }
        telemetry.addData("Turret travel (from the two ends)", "%.0f°  (assumes 3.3 V = one servo turn)", turretTravelDeg());
        telemetry.addData("Sweep (Y)", sweepResult);
        telemetry.addLine("Stick creep | dpad step | A/B/X = 0/1/0.5 | LB/RB save ends | Y sweep");
    }

    /** Turret degrees covered between the two saved ends, assuming 0..3.3 V is one full servo turn. */
    private double turretTravelDeg() {
        return Math.abs(voltsAt1 - voltsAt0) / ENCODER_MAX_VOLTS * 360.0 * TURRET_REVS_PER_SERVO_REV;
    }
}
