package org.firstinspires.ftc.teamcode.decode.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@TeleOp(name = "!!!SpindexerTest")
public class NewSpinTest extends LinearOpMode {

    private DcMotorEx spinMotor;
    int[] outtakePositions = {0,250,500};
    int[] intakePositions = {125, 375, 625};
    boolean intakeActive = false;
    int index = 0;

    boolean wasRightBumperPressed = false;

    boolean wasLeftBumperPressed = false;

    private NormalizedColorSensor intakeColor;

    private float gain = 20;
    private enum Ball {EMPTY, PURPLE, GREEN}

    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};


    @Override
    public void runOpMode() throws InterruptedException {

        spinMotor = hardwareMap.get(DcMotorEx.class, "spinMotor");

        // Encoder setup
        spinMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ;
        spinMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set direction (flip if needed)
        spinMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");

        intakeColor.setGain(gain);

        enableLight(intakeColor);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper && !wasRightBumperPressed) {
                intakeNext(index);
            }

            if (gamepad1.left_bumper && !wasLeftBumperPressed) {
                intakePrev(index);
            }

            if (gamepad1.x) {
                outtake(0);
            } else if (gamepad1.y) {
                outtake(1);
            } else if (gamepad1.b) {
                outtake(2);
            }

            if (gamepad1.right_trigger >= 0.3F) {
                if (slots[0] == Ball.GREEN) {
                    outtake(0);
                } else if (slots[1] == Ball.GREEN) {
                    outtake(1);
                } else if (slots[2] == Ball.GREEN) {
                    outtake(2);
                }
            }

            if (gamepad1.left_trigger >= 0.3F) {
                if (slots[0] == Ball.PURPLE) {
                    outtake(0);
                } else if (slots[1] == Ball.PURPLE) {
                    outtake(1);
                } else if (slots[2] == Ball.PURPLE) {
                    outtake(2);
                }
            }

            wasRightBumperPressed = gamepad1.right_bumper;
            wasLeftBumperPressed = gamepad1.left_bumper;

            if (gamepad1.a) {
                Ball color = detectColor(intakeColor);
                if (color == Ball.EMPTY)
                    telemetry.addLine("Empty");
                else if (color == Ball.GREEN)
                    telemetry.addLine("Green");
                else if (color == Ball.PURPLE)
                    telemetry.addLine("Purple");

                if (intakeActive && !spinMotor.isBusy()) {
                    Ball currentDetection = detectColor(intakeColor);
                    if (currentDetection != Ball.EMPTY)
                        slots[index] = currentDetection;
                }
            }

            telemetry.addLine("Slots");

            for (Ball curr : slots) {
                if (curr == Ball.EMPTY)
                    telemetry.addLine("Empty");
                else if (curr == Ball.GREEN)
                    telemetry.addLine("Green");
                else if (curr == Ball.PURPLE)
                    telemetry.addLine("Purple");
            }

            telemetry.addData("Current Position", spinMotor.getCurrentPosition());
            telemetry.addData("Index", index);
            telemetry.addData("Launch Velocity", spinMotor.getVelocity());
            telemetry.addData("Launch Power", spinMotor.getPower());
            telemetry.update();
        }
    }

    public int nextIndex(int curr) {
        curr += 1;
        curr %= 3;
        return curr;
    }

    public int prevIndex(int curr) {
        curr += 2;
        curr %= 3;
        return curr;
    }

    public int closestModular(int mod, int value) {
        int n = 0;
        while (Math.abs(750*n+mod-value) > 375) {
            n += 1;
        }
        return 750*n+mod;
    }

    public void rotateToIndex(int target) {
        index = target;
        int targetPosition;
        if (intakeActive) {
            targetPosition = intakePositions[target];
        } else {
            targetPosition = outtakePositions[target];
        }

        spinMotor.setTargetPosition(closestModular(targetPosition, spinMotor.getCurrentPosition()));
        spinMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.25);
    }

    public void rotateNext(int curr) {
        index = nextIndex(curr);
        rotateToIndex(index);
    }

    public void rotatePrev(int curr) {
        index = prevIndex(curr);
        rotateToIndex(index);
    }

    public void outtake(int target) {
        intakeActive = false;
        rotateToIndex(target);
    }

    public void intake(int target) {
        intakeActive = true;
        rotateToIndex(target);
    }

    public void intakeNext(int curr) {
        intake(nextIndex(curr));
    }

    public void intakePrev(int curr) {
        intake(prevIndex(curr));
    }

    public void outtakeNext(int curr) {
        outtake(nextIndex(curr));
    }

    public void outtakePrev(int curr) {
        outtake(prevIndex(curr));
    }

    // ===== INTERNAL SENSORS/COLOR =====
    private Ball detectColor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;
        float tol = 0.20f;

        if (b > r*(1+tol) && b > g*(1+tol)) return Ball.PURPLE;
        if (g > r*(1+tol) && g > b*(1+tol)) return Ball.GREEN;

//        if (r > 0.01 || g > 0.01 || b > 0.01) return Ball.EMPTY;
        return Ball.EMPTY;

    }

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) ((SwitchableLight) s).enableLight(true);
    }
}