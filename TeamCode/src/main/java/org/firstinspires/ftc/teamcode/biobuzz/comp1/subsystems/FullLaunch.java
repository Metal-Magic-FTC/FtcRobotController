package org.firstinspires.ftc.teamcode.biobuzz.comp1.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.LauncherConstants;

/**
 * Intake + transfer (same as Transfer) plus the flywheel on the hub's velocity PIDF, with live P/F tuning.
 *
 * gamepad1 - intake / transfer (unchanged from Transfer)
 *   right trigger  - run intake + transfer in      left trigger - run them out
 *   A = power 1    B = power 0    dpad up/down = power +/- 0.1    X = flip transfer direction
 * gamepad1 - flywheel
 *   right bumper   - flywheel on / off
 *   Y              - next target velocity: 900 -> 1500 -> 1700 -> 1800 -> 1900 -> 2000 -> 2100 -> 2400 -> 900 ...
 *   While the flywheel is on, the TRANSFER waits (intake still runs) until the flywheel is at speed, so a ball is
 *   never fed into a slow wheel.
 *
 * gamepad2 - PIDF tuning (velocities in encoder ticks per second)
 *   dpad up/down    - P +/- step        dpad right/left - F +/- step
 *   B               - cycle step size
 *   HOW TO TUNE: P = 0 first, raise F until the flywheel settles just under the target at HIGH, then check LOW.
 *   Then raise P until it recovers fast after a shot without oscillating. Copy the numbers into LauncherConstants.
 */
@TeleOp(name = "FullLaunch", group = "BioBuzz Tests")
public class FullLaunch extends OpMode {

    public DcMotor intake = null;
    public DcMotor transfer = null;
    public DcMotorEx launchMotor = null;
    double power_intake = 0;
    double sign_intake = 1;

    // ======================= FLYWHEEL =======================
    /** Y steps through these in order, then wraps back to the first. ticks/s (6000 rpm, 28 ticks/rev -> ~2800 max). */
    public static final double[] VELOCITIES = {900, 1500, 1700, 1800, 1900, 2000, 2100, 2400};

    boolean flywheelOn = false;
    int velocityIndex = 0;
    double targetVelocity = VELOCITIES[velocityIndex];

    // Start from the known-sensible values instead of 0 (P = F = 0 means the hub outputs nothing).
    double P = LauncherConstants.VEL_P;
    double F = LauncherConstants.VEL_F;
    double sentP = Double.NaN, sentF = Double.NaN;
    PIDFCoefficients onHub = new PIDFCoefficients(8, 0, 0, 20); // read back from the hub after each send

    final double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.dcMotor.get("transferMotor");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setDirection(LauncherConstants.MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down, don't brake the flywheel
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sendPidfIfChanged();
        launchMotor.setVelocity(0);
    }

    @Override
    public void loop() {
        // ----- flywheel controls -----
        if (gamepad1.rightBumperWasPressed()) flywheelOn = !flywheelOn;
        if (gamepad1.yWasPressed()) {
            velocityIndex = (velocityIndex + 1) % VELOCITIES.length;
            targetVelocity = VELOCITIES[velocityIndex];
        }

        // ----- PIDF tuning (gamepad2) -----
        if (gamepad2.bWasPressed()) stepIndex = (stepIndex + 1) % stepSizes.length;
        double step = stepSizes[stepIndex];
        if (gamepad2.dpadUpWasPressed()) P += step;
        if (gamepad2.dpadDownWasPressed()) P -= step;
        if (gamepad2.dpadRightWasPressed()) F += step;
        if (gamepad2.dpadLeftWasPressed()) F -= step;
        P = Math.max(0, P); // negative gains make it run away
        F = Math.max(0, F);
        sendPidfIfChanged();

        double commanded = flywheelOn ? targetVelocity : 0;
        launchMotor.setVelocity(commanded);
        double velocity = launchMotor.getVelocity();
        double error = commanded - velocity;
        boolean atSpeed = flywheelOn
                && Math.abs(error) <= LauncherConstants.VELOCITY_TOLERANCE * targetVelocity;

        // ----- intake / transfer (gamepad1, same as Transfer) -----
        // With the flywheel on, hold the transfer until it is at speed so no ball goes into a slow wheel.
//        boolean transferAllowed = !flywheelOn || atSpeed;
        if (gamepad1.right_trigger >= 0.3F) {
            intake.setPower(power_intake);
            transfer.setPower(sign_intake * power_intake);

        }
        else if (gamepad1.left_trigger >= 0.3F) {
            intake.setPower(-power_intake);
            transfer.setPower(-sign_intake * power_intake); // reversing out is always allowed
        }
        else {
            intake.setPower(0);
            transfer.setPower(0);
        }

        if (gamepad1.aWasPressed()) {
            power_intake = 1;
        }
        if (gamepad1.bWasPressed()) {
            power_intake = 0;
        }
        if (gamepad1.dpadUpWasPressed()) {
            power_intake += 0.1;
        }
        if (gamepad1.dpadDownWasPressed()) {
            power_intake -= 0.1;
        }
        power_intake = Range.clip(power_intake, 0, 1);
        if (gamepad1.xWasPressed()) {
            sign_intake *= -1;
        }

        // ----- telemetry -----
        telemetry.addData("Flywheel (RB)", flywheelOn ? (atSpeed ? "ON - READY" : "ON - spinning up") : "OFF");
        telemetry.addData("Target (Y)", "%.0f ticks/s  (%d of %d)", targetVelocity,
                velocityIndex + 1, VELOCITIES.length);
        telemetry.addData("Velocity", "%.0f ticks/s", velocity);
        telemetry.addData("Error", "%.0f ticks/s", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("P (gp2 dpad U/D)", "%.4f   hub has %.4f", P, onHub.p);
        telemetry.addData("F (gp2 dpad L/R)", "%.4f   hub has %.4f", F, onHub.f);
        telemetry.addData("Step (gp2 B)", "%.4f", step);
        telemetry.addLine("-----------------------------");
        telemetry.addData("motor speed", power_intake);
        telemetry.addData("sign", sign_intake);
        if (flywheelOn && !atSpeed && gamepad1.right_trigger >= 0.3F) {
            telemetry.addLine("Transfer waiting for flywheel");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        launchMotor.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    /** Only talk to the hub when P or F actually changed; resending every loop slows the loop for nothing. */
    private void sendPidfIfChanged() {
        if (P == sentP && F == sentF) return;
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));
        sentP = P;
        sentF = F;
        onHub = launchMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
