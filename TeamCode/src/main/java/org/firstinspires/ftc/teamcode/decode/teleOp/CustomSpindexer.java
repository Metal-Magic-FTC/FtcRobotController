package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class CustomSpindexer {

    // ---------------------------------------------------
    // ENUM
    // ---------------------------------------------------
    public enum BallColor {
        PURPLE, GREEN, EMPTY, UNKNOWN
    }

    // ---------------------------------------------------
    // HARDWARE
    // ---------------------------------------------------
    private DcMotor spinMotor;
    private Servo flickServo;

    private NormalizedColorSensor backColor;
    private NormalizedColorSensor leftColor;
    private NormalizedColorSensor rightColor;

    // ---------------------------------------------------
    // STATE
    // ---------------------------------------------------
    public BallColor[] balls = new BallColor[3];
    public int index = 0;
    private int currentTarget = 0;

    // ---------------------------------------------------
    // CONSTANTS
    // ---------------------------------------------------
    public final int[] POSITIONS = {-30, 217, 485};
    public final int[] INTAKE_POSITIONS = {352, -115, 142};

    private final float gain = 20;
    private final float tol = 0.20f;

    // ---------------------------------------------------
    // INIT
    // ---------------------------------------------------
    public CustomSpindexer(HardwareMap hardwareMap,
                           String spinMotorName,
                           String flickServoName,
                           String backColorName,
                           String leftColorName,
                           String rightColorName) {

        // Motor
        spinMotor = hardwareMap.get(DcMotor.class, spinMotorName);
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        // Servo
        flickServo = hardwareMap.servo.get(flickServoName);

        // Color sensors
        backColor  = hardwareMap.get(NormalizedColorSensor.class, backColorName);
        leftColor  = hardwareMap.get(NormalizedColorSensor.class, leftColorName);
        rightColor = hardwareMap.get(NormalizedColorSensor.class, rightColorName);

        backColor.setGain(gain);
        leftColor.setGain(gain);
        rightColor.setGain(gain);

        enableIfLight(backColor);
        enableIfLight(leftColor);
        enableIfLight(rightColor);

        resetBallArray();
    }

    private void enableIfLight(NormalizedColorSensor sensor) {
        if (sensor instanceof SwitchableLight)
            ((SwitchableLight) sensor).enableLight(true);
    }

    private void resetBallArray() {
        balls[0] = BallColor.EMPTY;
        balls[1] = BallColor.EMPTY;
        balls[2] = BallColor.EMPTY;
    }

    // ---------------------------------------------------
    // MOVEMENT
    // ---------------------------------------------------
    public void moveToIndex(int newIndex, int[] table, double power) {
        currentTarget = table[newIndex];
        spinMotor.setTargetPosition(currentTarget);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(power);
    }

    // ---------------------------------------------------
    // BALL SEARCHING
    // ---------------------------------------------------
    public int findClosestColor(BallColor target, int currentIndex) {
        for (int offset = 0; offset < balls.length; offset++) {
            int right = (currentIndex + offset) % balls.length;
            int left  = (currentIndex - offset + balls.length) % balls.length;

            if (balls[right] == target) return right;
            if (balls[left] == target)  return left;
        }
        return currentIndex;
    }

    public int findClosestEmpty(int currentIndex) {
        return findClosestColor(BallColor.EMPTY, currentIndex);
    }

    // ---------------------------------------------------
    // COLOR DETECTION
    // ---------------------------------------------------
    private BallColor detectBallColor(NormalizedColorSensor sensor) {

        NormalizedRGBA c = sensor.getNormalizedColors();

        float r = c.red;
        float g = c.green;
        float b = c.blue;

        if (b > r * (1 + tol) && b > g * (1 + tol)) return BallColor.PURPLE;
        if (g > r * (1 + tol) && g > b * (1 + tol)) return BallColor.GREEN;

        if (r > 0.01 || g > 0.01 || b > 0.01) return BallColor.UNKNOWN;
        return BallColor.EMPTY;
    }

    // Scan all 3 slots
    public void scanAllBalls() {
        balls[0] = detectBallColor(backColor);
        balls[1] = detectBallColor(rightColor);
        balls[2] = detectBallColor(leftColor);
    }

    // ---------------------------------------------------
    // FIRING
    // ---------------------------------------------------
    public void launchBallAt(int slot) {
        if (balls[slot] != BallColor.EMPTY) {

            // Flicker
            flickServo.setPosition(0.6);
            sleep(200);
            flickServo.setPosition(1);

            balls[slot] = BallColor.EMPTY;
        }
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (Exception ignored) {}
    }

    // ---------------------------------------------------
    // TELEMETRY HELPERS
    // ---------------------------------------------------
    public int getEncoder() {
        return spinMotor.getCurrentPosition();
    }
}

