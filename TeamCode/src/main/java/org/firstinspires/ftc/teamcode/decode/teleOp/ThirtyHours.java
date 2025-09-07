package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="!Thirty dirty hours")
public class ThirtyHours extends LinearOpMode {

    DcMotor leftLaunch = null;
    DcMotor rightLaunch = null;
    DcMotor intakeMotor = null;

    public Servo launchGate = null;

    public static double GATE_OPEN = 1;
    public static double GATE_CLOSE = 0.85;
    public static double GATE_LAUNCH = 0.55;

    private CustomMecanumDrive drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(); // initializing everything

        waitForStart(); // waiting until driver clicks play button

        boolean launchProcedure = false;
        double timeMilli = 0;

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drivetrain.driveMecanum(strafe, drive, turn);

            boolean INTAKE_CONTROL = gamepad1.left_trigger >= 0.3F;
            // boolean GATE_CONTROL = gamepad1.a;
            boolean LAUNCH_CONTROL = gamepad1.right_trigger >= 0.3F;


            if (launchProcedure) {

                launchGate.setPosition(GATE_LAUNCH);
                leftLaunch.setPower(1);
                rightLaunch.setPower(1);

                if (!LAUNCH_CONTROL) {
                    launchProcedure = false;
                }
            } else {
                intake(INTAKE_CONTROL);

                if (LAUNCH_CONTROL) {
                    launchProcedure = true;
                    timeMilli = System.currentTimeMillis();
                    launchGate.setPosition(GATE_LAUNCH);
                }

                leftLaunch.setPower(0);
                rightLaunch.setPower(0);

            }

            telemetry.addData("Servo Position: ", launchGate.getPosition());
            telemetry.update();

        }
    }

    public static boolean elapsedTime(double timeTarget, double elapsedTarget) {

        return System.currentTimeMillis() - timeTarget >= elapsedTarget;

    }

    public void intake(boolean INTAKE_CONTROL) {
        if (INTAKE_CONTROL) {
            // intake
            intakeMotor.setPower(1);
            launchGate.setPosition(GATE_OPEN);

        } else {
            intakeMotor.setPower(0);
            launchGate.setPosition(GATE_CLOSE);
        }

    }

    public void initialize() {

        leftLaunch = hardwareMap.get(DcMotor.class, "leftLaunch");
        rightLaunch = hardwareMap.get(DcMotor.class, "rightLaunch");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        leftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLaunch.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        launchGate = hardwareMap.servo.get("leftGate");
        launchGate.setPosition(1);

        // fl - 0, fr - 1, bl - 2, br - 3
        drivetrain = new CustomMecanumDrive(hardwareMap);

    }

}
