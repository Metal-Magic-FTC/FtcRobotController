package org.firstinspires.ftc.teamcode.biobuzz.comp1.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.LauncherConstants;

/**
 * Launcher flywheel P / F tuner ("launchMotor"), same idea as the tuning video: switch between a HIGH and a LOW
 * target and adjust F then P until the wheel gets to each target fast and holds it. Velocities are encoder ticks/s
 * (what setVelocity uses), the same units as the TeleOp launcher speeds (1600 / 1650 / 1700).
 *
 * Starts with P = 0, F = 0 and the flywheel OFF.
 *
 * gamepad1:
 *   A               - flywheel on / off
 *   Y               - switch target: HIGH <-> LOW
 *   right bumper    - MEASURE F automatically: runs full power for 2 s, then F = 32767 / top speed (REV's method)
 *   dpad right/left - F + / - step
 *   dpad up/down    - P + / - step
 *   B               - change step size (10, 1, 0.1, 0.01, 0.001)
 *   LEFT TRIGGER    - SHOOT (hold): transfer feeds up into the launcher (otherwise it spins down, like the TeleOp)
 *   RIGHT TRIGGER   - intake on / off      X - clear the shot dip stats
 *
 * SHOT RECOVERY (the real test for multi-ball shooting): load several balls and hold LEFT TRIGGER. For every ball,
 * telemetry shows how far the wheel DROPPED and how long it took to get back within 3% of the target.
 * Raise P until the dip and the recovery time stop improving or the speed starts overshooting / bouncing.
 *
 * HOW TO TUNE
 *   1. Robot on blocks / nothing touching the wheel. A to turn it on.
 *   2. Right bumper to measure F (or raise F by hand with P = 0 until "Current" sits just under the target).
 *   3. With F set, raise P until the wheel snaps to the new target when you press Y, without bouncing above it.
 *      If it overshoots and wobbles, lower P.
 *   4. Feed a ball (or press Y) and watch how fast it recovers. Copy P and F into the TeleOp (LAUNCH_P / LAUNCH_F).
 */
@TeleOp(name = "BioBuzz Flywheel Recovery Tuner", group = "BioBuzz Tests")
public class FlywheelRecoveryTuner extends OpMode {

    private static final double HIGH_VELOCITY = 1700;
    private static final double LOW_VELOCITY = 1200;
    private static final double[] STEP_SIZES = {10.0, 1.0, 0.1, 0.01, 0.001};
    /** Same signs as the TeleOp: intake + = in, transfer - = up into the launcher. */
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_FEED_POWER = -1.0;
    private static final double TRANSFER_DOWN_POWER = 0.5;
    /** A shot dip starts when speed falls this far below target, and is over when it is back within RECOVERED. */
    private static final double DIP_START_FRACTION = 0.05;
    private static final double RECOVERED_FRACTION = 0.03;

    private DcMotor intake, transfer;
    private boolean intakeOn = false, rightTriggerWasDown = false;

    // shot dip tracking
    private boolean inDip = false;
    private double dipMin = 0;
    private final ElapsedTime dipTimer = new ElapsedTime();
    private int shots = 0;
    private double lastDrop = Double.NaN, lastRecoveryMs = Double.NaN, worstDrop = 0, worstRecoveryMs = 0;

    private DcMotorEx launchMotor;
    private double target = HIGH_VELOCITY;
    private boolean on = false;
    private double P = 0, F = 0;
    private double sentP = Double.NaN, sentF = Double.NaN;
    private int stepIndex = 1;

    // automatic F measurement
    private boolean measuring = false;
    private final ElapsedTime measureTimer = new ElapsedTime();
    private double measuredMax = 0;
    private String measureResult = "not run (right bumper)";

    @Override
    public void init() {
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setDirection(LauncherConstants.MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD); // same as the TeleOp
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast, don't brake the wheel
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sendPidfIfChanged();
        launchMotor.setVelocity(0);

        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer = hardwareMap.dcMotor.get("transferMotor");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addLine("Flywheel tuner ready. Wheel is OFF - press A to start.");
    }

    @Override
    public void loop() {
        if (measuring) {
            runMeasurement();
        } else {
            if (gamepad1.aWasPressed()) on = !on;
            if (gamepad1.yWasPressed()) target = target == HIGH_VELOCITY ? LOW_VELOCITY : HIGH_VELOCITY;
            if (gamepad1.bWasPressed()) stepIndex = (stepIndex + 1) % STEP_SIZES.length;
            double step = STEP_SIZES[stepIndex];
            if (gamepad1.dpadRightWasPressed()) F += step;
            if (gamepad1.dpadLeftWasPressed()) F -= step;
            if (gamepad1.dpadUpWasPressed()) P += step;
            if (gamepad1.dpadDownWasPressed()) P -= step; // (the video checks dpad UP twice here - fixed)
            P = Math.max(0, P);
            F = Math.max(0, F);
            if (gamepad1.rightBumperWasPressed()) startMeasurement();

            sendPidfIfChanged();
            launchMotor.setVelocity(on ? target : 0);
        }

        // intake / transfer, same as the TeleOp
        boolean rightDown = gamepad1.right_trigger > 0.5;
        if (rightDown && !rightTriggerWasDown) intakeOn = !intakeOn;
        rightTriggerWasDown = rightDown;
        intake.setPower(intakeOn ? INTAKE_POWER : 0);
        boolean shooting = gamepad1.left_trigger > 0.5;
        transfer.setPower(shooting ? TRANSFER_FEED_POWER : TRANSFER_DOWN_POWER);
        if (gamepad1.xWasPressed()) {
            shots = 0;
            lastDrop = lastRecoveryMs = Double.NaN;
            worstDrop = worstRecoveryMs = 0;
        }

        double velocity = launchMotor.getVelocity();
        trackDip(velocity);
        double error = (on ? target : 0) - velocity;
        telemetry.addData("Flywheel (A)", measuring ? "MEASURING F..." : on ? "ON" : "OFF");
        telemetry.addData("Target (Y)", "%.0f ticks/s  (%s)", target, target == HIGH_VELOCITY ? "HIGH" : "LOW");
        telemetry.addData("Current velocity", "%.1f", velocity);
        telemetry.addData("Error", "%.1f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step size", "%.4f (B)", STEP_SIZES[stepIndex]);
        telemetry.addData("F measure (RB)", measureResult);
        telemetry.addLine("----- shot recovery (hold LT with balls loaded, X clears) -----");
        telemetry.addData("Shots seen", shots);
        telemetry.addData("Last shot", Double.isNaN(lastDrop) ? "-" : String.format("dropped %.0f ticks/s (%.0f%%), back in %.0f ms",
                lastDrop, 100 * lastDrop / target, lastRecoveryMs));
        telemetry.addData("Worst", String.format("dropped %.0f ticks/s, slowest recovery %.0f ms", worstDrop, worstRecoveryMs));
        telemetry.addData("Intake (RT)", intakeOn ? "ON" : "off");
        telemetry.addData("Transfer (LT)", shooting ? "FEEDING UP" : "down");
    }

    /** Detects each ball's speed dip and times how long the wheel takes to recover. */
    private void trackDip(double velocity) {
        if (!on || measuring) {
            inDip = false;
            return;
        }
        if (!inDip && velocity < target * (1 - DIP_START_FRACTION)) {
            inDip = true;
            dipMin = velocity;
            dipTimer.reset();
        }
        if (inDip) {
            dipMin = Math.min(dipMin, velocity);
            if (velocity >= target * (1 - RECOVERED_FRACTION)) {
                inDip = false;
                shots++;
                lastDrop = target - dipMin;
                lastRecoveryMs = dipTimer.milliseconds();
                worstDrop = Math.max(worstDrop, lastDrop);
                worstRecoveryMs = Math.max(worstRecoveryMs, lastRecoveryMs);
            }
        }
    }

    @Override
    public void stop() {
        launchMotor.setVelocity(0);
        launchMotor.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    private void startMeasurement() {
        measuring = true;
        measuredMax = 0;
        measureTimer.reset();
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setPower(1.0);
    }

    private void runMeasurement() {
        double v = Math.abs(launchMotor.getVelocity());
        if (measureTimer.seconds() > 1.0) measuredMax = Math.max(measuredMax, v); // skip the spin-up
        if (measureTimer.seconds() < 2.0) return;

        launchMotor.setPower(0);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        measuring = false;
        on = false;
        if (measuredMax < 100) {
            measureResult = String.format("FAILED - top speed only %.0f ticks/s (encoder plugged in?)", measuredMax);
            return;
        }
        F = 32767.0 / measuredMax;
        sentF = Double.NaN; // force a resend
        measureResult = String.format("top speed %.0f ticks/s -> F = %.2f", measuredMax, F);
    }

    /** Only send to the hub when P or F changed (sending every loop slows the loop for nothing). */
    private void sendPidfIfChanged() {
        if (P == sentP && F == sentF) return;
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));
        sentP = P;
        sentF = F;
    }
}
