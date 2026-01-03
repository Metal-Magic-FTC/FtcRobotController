package org.firstinspires.ftc.teamcode.decode.teleOp.comptwotests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@TeleOp(name = "!!!!!SpindexerV2")
public class SpindexerV2 extends LinearOpMode {

    private DcMotor spinMotor;
    private NormalizedColorSensor intakeColor;

    private static final int[] OUTTAKE_POS = {0, 250, 500};
    private static final int[] INTAKE_POS  = {125, 375, 625};

    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    private int index = 0;
    private boolean intakeActive = false;
    private boolean waitingForBall = false;

    private float gain = 20;

    // Button edge detection
    private boolean prevA, prevX, prevY, prevB;

    @Override
    public void runOpMode() throws InterruptedException {

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");

        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeColor.setGain(gain);
        enableLight(intakeColor);

        waitForStart();

        while (opModeIsActive()) {

            boolean intakePressed      = gamepad1.a && !prevA;
            boolean aimGreenPressed    = gamepad1.x && !prevX;
            boolean aimPurplePressed   = gamepad1.y && !prevY;
            boolean shootPressed       = gamepad1.b && !prevB;

            prevA = gamepad1.a;
            prevX = gamepad1.x;
            prevY = gamepad1.y;
            prevB = gamepad1.b;

            // intake
            if (intakePressed) {
                int nextEmpty = findNextEmpty();
                if (nextEmpty != -1) {
                    intakeActive = true;
                    waitingForBall = true;
                    rotateToIndex(nextEmpty);
                }
            }

            // go to balls
            if (aimGreenPressed) {
                aimClosest(Ball.GREEN);
            }

            if (aimPurplePressed) {
                aimClosest(Ball.PURPLE);
            }

            // shoot
            if (shootPressed) {
                if (slots[index] != Ball.EMPTY) {
                    // INSERT TS launcher code
                    slots[index] = Ball.EMPTY;

                    int nextLoaded = findClosestLoaded();
                    if (nextLoaded != -1) {
                        rotateToIndex(nextLoaded);
                    }
                }
            }

            // spindexer logic tsts

            if (waitingForBall && intakeActive && !spinMotor.isBusy()) {
                Ball detected = detectColor(intakeColor);
                if (detected != Ball.EMPTY) {
                    slots[index] = detected;
                    waitingForBall = false;

                    int nextEmpty = findNextEmpty();
                    if (nextEmpty != -1) {
                        rotateToIndex(nextEmpty);
                        waitingForBall = true;
                    }
                }
            }


            telemetry.addData("Index", index);
            telemetry.addData("Position", spinMotor.getCurrentPosition());
            telemetry.addLine("Slots:");
            for (int i = 0; i < 3; i++) {
                telemetry.addData("Slot " + i, slots[i]);
            }
            telemetry.update();
        }
    }

    // ROTATION

    private void rotateToIndex(int target) {
        index = target;
        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
        int targetPos = closestModular(base, spinMotor.getCurrentPosition());

        spinMotor.setTargetPosition(targetPos);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.3);
    }

    private int closestModular(int mod, int current) {
        int best = mod;
        int minDiff = Integer.MAX_VALUE;
        for (int k = -2; k <= 2; k++) {
            int candidate = mod + 750 * k;
            int diff = Math.abs(candidate - current);
            if (diff < minDiff) {
                minDiff = diff;
                best = candidate;
            }
        }
        return best;
    }

    private int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == Ball.EMPTY) return idx;
        }
        return -1;
    }

    private int findClosestLoaded() {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] != Ball.EMPTY) return idx;
        }
        return -1;
    }

    private void aimClosest(Ball target) {
        intakeActive = false;
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == target) {
                rotateToIndex(idx);
                return;
            }
        }
    }

    // COLOR SENSORRRR

    private Ball detectColor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;
        float tol = 0.20f;

        if (b > r * (1 + tol) && b > g * (1 + tol)) return Ball.PURPLE;
        if (g > r * (1 + tol) && g > b * (1 + tol)) return Ball.GREEN;
        return Ball.EMPTY;
    }

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) {
            ((SwitchableLight) s).enableLight(true);
        }
    }
}