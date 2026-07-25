package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "!!!!!!`Individual Wheel Test", group = "Test")
public class IndividualWheelTest extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Index of the currently selected wheel: 0=FL, 1=FR, 2=BL, 3=BR, -1=none selected
    private int selectedWheel = -1;
    private final String[] wheelNames = {"Front Left", "Front Right", "Back Left", "Back Right"};

    // Edge-detection helpers for dpad buttons so a single press = one toggle
    private boolean dpadUpPrev, dpadDownPrev, dpadLeftPrev, dpadRightPrev;

    @Override
    public void runOpMode() {

        // Map motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Match directions from CustomMecanumDrive
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight};

        telemetry.addLine("Individual Wheel Test Ready");
        telemetry.addLine("Dpad Up/Right/Down/Left = FL/FR/BR/BL");
        telemetry.addLine("Left Stick Y = drive selected wheel");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---- Dpad selection (toggle: press again to deselect) ----
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;

            if (dpadUp && !dpadUpPrev) toggleSelection(0);
            if (dpadRight && !dpadRightPrev) toggleSelection(1);
            if (dpadDown && !dpadDownPrev) toggleSelection(3); // Back Right
            if (dpadLeft && !dpadLeftPrev) toggleSelection(2); // Back Left

            dpadUpPrev = dpadUp;
            dpadRightPrev = dpadRight;
            dpadDownPrev = dpadDown;
            dpadLeftPrev = dpadLeft;

            // ---- Drive only the selected wheel ----
            double power = -gamepad1.left_stick_y; // negative = forward on most gamepads

            for (int i = 0; i < motors.length; i++) {
                if (i == selectedWheel) {
                    motors[i].setPower(power);
                } else {
                    motors[i].setPower(0);
                }
            }

            // ---- Telemetry ----
            telemetry.addData("Selected Wheel", selectedWheel == -1 ? "None" : wheelNames[selectedWheel]);
            telemetry.addData("Stick Power", "%.2f", power);
            telemetry.addLine("---- Wheel Power ----");
            for (int i = 0; i < motors.length; i++) {
                telemetry.addData(wheelNames[i], "%.2f", motors[i].getPower());
            }
            telemetry.update();
        }
    }

    // Selecting the same wheel again deselects it (and stops it)
    private void toggleSelection(int wheelIndex) {
        if (selectedWheel == wheelIndex) {
            selectedWheel = -1;
        } else {
            selectedWheel = wheelIndex;
        }
    }
}