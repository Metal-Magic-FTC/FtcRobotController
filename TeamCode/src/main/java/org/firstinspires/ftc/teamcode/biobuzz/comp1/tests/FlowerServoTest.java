package org.firstinspires.ftc.teamcode.biobuzz.comp1.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Ramp servo test ("flowerServo", configured as a Servo). Find the positions you want, then copy them into code.
 *
 * The servo does NOT move at INIT - it only moves once you press a button after START.
 *
 * gamepad1:
 *   A                 - position 0
 *   B                 - position 1
 *   dpad up / down    - position + / - step
 *   dpad right / left - bigger / smaller step (0.1, 0.05, 0.01, 0.005, 0.001)
 */
@TeleOp(name = "Flower Ramp Servo Test", group = "BioBuzz Tests")
public class FlowerServoTest extends OpMode {

    private static final double[] STEPS = {0.1, 0.05, 0.01, 0.005, 0.001};

    private Servo servo;
    private double position = 0.5;
    private boolean commanded = false;
    private int stepIndex = 2; // 0.01

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "flowerServo");
        telemetry.addLine("Flower ramp servo ready - it moves on the first button press after START.");
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed() && stepIndex > 0) stepIndex--;           // bigger step
        if (gamepad1.dpadLeftWasPressed() && stepIndex < STEPS.length - 1) stepIndex++; // smaller step
        double step = STEPS[stepIndex];

        double before = position;
        if (gamepad1.aWasPressed()) { position = 0; commanded = true; }
        if (gamepad1.bWasPressed()) { position = 1; commanded = true; }
        if (gamepad1.dpadUpWasPressed()) { position += step; commanded = true; }
        if (gamepad1.dpadDownWasPressed()) { position -= step; commanded = true; }
        position = Range.clip(position, 0, 1);

        if (commanded) servo.setPosition(position);

        telemetry.addData("Position", "%.3f%s", position, commanded ? "" : "  (not sent yet)");
        telemetry.addData("Step (dpad L/R)", "%.3f", step);
        telemetry.addLine("A = 0 | B = 1 | dpad up/down = +/- step");
    }
}
