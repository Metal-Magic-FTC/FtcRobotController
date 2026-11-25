package org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.advanced;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * STEP 3 of 3 - kP, using the FULL controller (Flywheel.setMotorRPM: kS + kV + kP, voltage compensation, dip boost).
 * Set Flywheel.KS and Flywheel.KV from steps 1 and 2 first.
 *
 * Toggle A / B between the HIGH and LOW goals and raise kP until the wheel gets to each goal fast without
 * overshooting much. If you HEAR it oscillate (a warbling hum), kP is too high - back off. Then test with real balls
 * (LEFT TRIGGER feeds) and compare the dip / recovery numbers with the boost on and off (X).
 * Type the final kP into Flywheel.KP.
 *
 * gamepad1:
 *   dpad up / down    - kP + / - step         dpad right / left - bigger / smaller step
 *   A / B             - goal = HIGH / LOW RPM (spin-up time is measured on each switch)
 *   X                 - dip boost on / off     Y - flywheel on / off (starts ON)
 *   LEFT TRIGGER      - hold to feed balls (transfer up)      RIGHT TRIGGER - intake on / off
 *   BACK              - clear the stats
 */
@TeleOp(name = "Flywheel kP Tuner (3 of 3)", group = "BioBuzz Flywheel")
public class FlywheelKpTuner extends OpMode {

    public static final double HIGH_RPM = FlywheelKvTuner.HIGH_RPM;
    public static final double LOW_RPM = FlywheelKvTuner.LOW_RPM;
    /** "At speed" = within this fraction of the goal (spin-up time and ball recovery). */
    private static final double AT_SPEED_FRACTION = 0.02;
    /** A ball dip starts when the wheel falls this far under the goal. */
    private static final double DIP_FRACTION = 0.05;

    /** Same signs as the TeleOp: intake + = in, transfer - = up into the launcher. */
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_FEED_POWER = -1.0;
    private static final double TRANSFER_DOWN_POWER = 0.5;

    private final Flywheel flywheel = new Flywheel();
    private DcMotor intake, transfer;
    private double goalRpm = HIGH_RPM;
    private boolean on = true;
    private boolean intakeOn = false, rightTriggerWasDown = false;
    private final double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    private int incrementIdx = 2; // start at 0.0001

    // spin-up timing
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private boolean timingSpinUp = true;
    private double lastSpinUpSec = Double.NaN;
    private double maxOvershoot = 0;

    // ball dips
    private boolean inDip = false;
    private double dipMin = 0;
    private final ElapsedTime dipTimer = new ElapsedTime();
    private int balls = 0;
    private double lastDrop = Double.NaN, lastRecoveryMs = Double.NaN, worstDrop = 0, worstRecoveryMs = 0;

    @Override
    public void init() {
        flywheel.init(hardwareMap);
        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer = hardwareMap.dcMotor.get("transferMotor");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start() {
        spinUpTimer.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed() && incrementIdx < increments.length - 1) incrementIdx++;
        else if (gamepad1.dpadLeftWasPressed() && incrementIdx > 0) incrementIdx--;
        double step = increments[incrementIdx];
        if (gamepad1.dpadUpWasPressed()) Flywheel.KP += step;
        if (gamepad1.dpadDownWasPressed()) Flywheel.KP = Math.max(0, Flywheel.KP - step);

        if (gamepad1.aWasPressed()) setGoal(HIGH_RPM);
        if (gamepad1.bWasPressed()) setGoal(LOW_RPM);
        if (gamepad1.xWasPressed()) Flywheel.BOOST_ENABLED = !Flywheel.BOOST_ENABLED;
        if (gamepad1.yWasPressed()) {
            on = !on;
            if (on) setGoal(goalRpm);
        }
        if (gamepad1.backWasPressed()) clearStats();

        if (on) flywheel.setMotorRPM(goalRpm);
        else flywheel.stopMotors();

        // intake / transfer for real-ball tests
        boolean rightDown = gamepad1.right_trigger > 0.5;
        if (rightDown && !rightTriggerWasDown) intakeOn = !intakeOn;
        rightTriggerWasDown = rightDown;
        intake.setPower(intakeOn ? INTAKE_POWER : 0);
        boolean feeding = gamepad1.left_trigger > 0.5;
        transfer.setPower(feeding ? TRANSFER_FEED_POWER : TRANSFER_DOWN_POWER);

        double rpm = flywheel.getRPM();
        if (on) measure(rpm);

        telemetry.addData("Step (dpad L/R)", "%.6f", step);
        telemetry.addData("kP (dpad U/D)", "%.6f", Flywheel.KP);
        telemetry.addData("kS / kV", "%.4f / %.7f", Flywheel.KS, Flywheel.KV);
        telemetry.addData("Goal (A high / B low)", "%.0f RPM  (%.0f ticks/s)", goalRpm, Flywheel.rpmToTicksPerSec(goalRpm));
        telemetry.addData("RPM", "%.0f   error %+.0f", rpm, goalRpm - rpm);
        telemetry.addData("Boost (X)", "%s%s", Flywheel.BOOST_ENABLED ? "ON" : "off",
                flywheel.isBoosting() ? "  <<< FULL POWER" : "");
        telemetry.addData("Power / battery", "%.2f / %.2f V", flywheel.lastPower(), flywheel.batteryVoltage());
        telemetry.addLine("----- spin-up (A / B switch) -----");
        telemetry.addData("Time to goal", timingSpinUp ? "timing..." : String.format("%.2f s", lastSpinUpSec));
        telemetry.addData("Overshoot", "%.0f RPM", maxOvershoot);
        telemetry.addLine("----- balls (hold LT, BACK clears) -----");
        telemetry.addData("Balls", balls);
        telemetry.addData("Last ball", Double.isNaN(lastDrop) ? "-" : String.format("dropped %.0f RPM, back in %.0f ms",
                lastDrop, lastRecoveryMs));
        telemetry.addData("Worst", "dropped %.0f RPM, slowest %.0f ms", worstDrop, worstRecoveryMs);
        telemetry.addData("Intake (RT) / feed (LT)", "%s / %s", intakeOn ? "ON" : "off", feeding ? "FEEDING" : "down");
    }

    private void setGoal(double rpm) {
        goalRpm = rpm;
        timingSpinUp = true;
        maxOvershoot = 0;
        inDip = false;
        spinUpTimer.reset();
    }

    private void measure(double rpm) {
        boolean atSpeed = Math.abs(goalRpm - rpm) <= AT_SPEED_FRACTION * goalRpm;
        if (timingSpinUp) {
            if (atSpeed) {
                timingSpinUp = false;
                lastSpinUpSec = spinUpTimer.seconds();
            }
            return; // the switch itself is not a ball dip
        }
        maxOvershoot = Math.max(maxOvershoot, rpm - goalRpm);

        if (!inDip && rpm < goalRpm * (1 - DIP_FRACTION)) {
            inDip = true;
            dipMin = rpm;
            dipTimer.reset();
        }
        if (inDip) {
            dipMin = Math.min(dipMin, rpm);
            if (atSpeed) {
                inDip = false;
                balls++;
                lastDrop = goalRpm - dipMin;
                lastRecoveryMs = dipTimer.milliseconds();
                worstDrop = Math.max(worstDrop, lastDrop);
                worstRecoveryMs = Math.max(worstRecoveryMs, lastRecoveryMs);
            }
        }
    }

    private void clearStats() {
        balls = 0;
        lastDrop = lastRecoveryMs = Double.NaN;
        worstDrop = worstRecoveryMs = 0;
        maxOvershoot = 0;
    }

    @Override
    public void stop() {
        flywheel.stopMotors();
        intake.setPower(0);
        transfer.setPower(0);
    }
}
