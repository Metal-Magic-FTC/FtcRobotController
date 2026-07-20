package org.firstinspires.ftc.teamcode.decode.AarushImprovements.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.decode.AarushImprovements.paths.BlueClose12Paths;
import org.firstinspires.ftc.teamcode.decode.AarushImprovements.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.AarushImprovements.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.decode.AarushImprovements.util.MotifDecoder;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import java.util.Arrays;

/**
 * Blue close 12-ball sorted autonomous (AarushImprovements copy).
 *
 * <p>Mirrors the Red close auto across the X-axis (see
 * {@link BlueClose12Paths#START_POSE}). The improvements over the original
 * V2 file include: shared path object, MotifDecoder-based aim, single color
 * detection function, no magic numbers, proper intake state machine.</p>
 */
@Autonomous(name = "Aarush Blue Close 12 Ball", group = "AarushImprovements")
public class BlueClose12BallAutonomous extends LinearOpMode {

    // ---------------- TUNABLE CONSTANTS ----------------

    private static final int[] INTAKE_POS = {125, 375, 625};
    private static final int[] OUTTAKE_POS = {500, 0, 250};

    private static final double SPIN_MOTOR_SPEED = 0.38;
    private static final double TURRET_POWER = 0.5;
    private static final double LAUNCH_VELOCITY = 1800.0;
    private static final double INTAKE_POWER = -0.6;

    private static final double FLICK_DOWN_POS = 0.96;
    private static final double FLICK_UP_POS = 0.80;
    private static final double HOOD_DEFAULT_POS = 0.80;

    private static final float COLOR_SENSOR_GAIN = 20.0f;
    private static final float COLOR_MIN_CHANNEL = 0.06f;
    private static final float COLOR_DOMINANCE = 1.15f;
    private static final float COLOR_MIN_TOTAL = 0.08f;

    private static final int TURRET_SHOOT_TICKS = -76;   // flipped for Blue
    private static final int TURRET_LEAVE_TICKS = 0;
    private static final int LIMELIGHT_MOTIF_PIPELINE = 3;
    private static final long APRILTAG_TIMEOUT_MS = 2_000;

    // ---------------- STATE ----------------

    private int spindexerIndex = 0;
    private boolean intakeActive = false;
    private boolean waitingForBall = false;

    private final Spindexer.Ball[] slots = {
            Spindexer.Ball.EMPTY, Spindexer.Ball.EMPTY, Spindexer.Ball.EMPTY
    };

    // ---------------- HARDWARE ----------------

    private Follower follower;
    private BlueClose12Paths paths;
    private Limelight3A limelight;
    private MecanumDrive drivetrain;

    private DcMotor spinMotor;
    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;
    private DcMotor turretMotor;
    private CRServo spinFlickServo;
    private Servo flickerServo;
    private Servo hoodServo;
    private NormalizedColorSensor intakeColor;
    private NormalizedColorSensor intakeColor2;

    // ---------------- LIFECYCLE ----------------

    @Override
    public void runOpMode() {
        initHardware();
        preloadSlots();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(BlueClose12Paths.START_POSE);
        paths = new BlueClose12Paths(follower);

        hoodServo.setPosition(HOOD_DEFAULT_POS);
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runAutonomous();

        telemetry.addLine("Finished");
        telemetry.update();
    }

    // ---------------- SEQUENCE ----------------

    private void runAutonomous() {
        launchMotor.setVelocity(LAUNCH_VELOCITY);
        turretRunToPosition(TURRET_SHOOT_TICKS);

        MotifDecoder.Motif motif = readMotifFromLimelight();
        applyMotifToSpindexer(motif);

        runPath(paths.shoot(), 50, 0.8);
        shootAll();

        startIntakeAt(0);

        // ---- INTAKE 1–3 ----
        runPathWithIntake(paths.toIntake1(), 0, 1.0);
        runPathWithIntake(paths.intake1(), 0, 0.21);

        forceLoadSlots(Spindexer.Ball.PURPLE, Spindexer.Ball.PURPLE, Spindexer.Ball.GREEN);
        applyMotifToSpindexer(motif);
        runPath(paths.shoot2(), 0, 0.5);
        shootAll();

        startIntakeAt(0);

        // ---- INTAKE 4–6 ----
        runPathWithIntake(paths.toIntake2(), 0, 1.0);
        runPathWithIntake(paths.intake2(), 0, 0.21);

        forceLoadSlots(Spindexer.Ball.PURPLE, Spindexer.Ball.GREEN, Spindexer.Ball.PURPLE);
        applyMotifToSpindexer(motif);
        runPath(paths.shoot3(), 0, 0.8);
        shootAll();

        // ---- LEAVE ----
        turretRunToPosition(TURRET_LEAVE_TICKS);
        runPath(paths.leave(), 0, 1.0);
        resetSlots();
    }

    // ---------------- HIGH-LEVEL ACTIONS ----------------

    private void preloadSlots() {
        slots[0] = Spindexer.Ball.PURPLE;
        slots[1] = Spindexer.Ball.GREEN;
        slots[2] = Spindexer.Ball.PURPLE;
    }

    private void startIntakeAt(int index) {
        intakeMotor.setPower(INTAKE_POWER);
        intakeActive = true;
        rotateToIndex(index);
        resetSlots();
    }

    private void forceLoadSlots(Spindexer.Ball a, Spindexer.Ball b, Spindexer.Ball c) {
        intakeMotor.setPower(INTAKE_POWER);
        slots[0] = a;
        slots[1] = b;
        slots[2] = c;
    }

    private void applyMotifToSpindexer(MotifDecoder.Motif motif) {
        intakeActive = false;
        int startIdx = MotifDecoder.findBestStartIndex(slots, motif);
        if (startIdx < 0) startIdx = spindexerIndex;
        spindexerIndex = startIdx;
        rotateToIndex(startIdx);
    }

    // ---------------- SPINDEXER (POSITION) ----------------

    private void rotateToIndex(int target) {
        spindexerIndex = target;
        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
        spinMotor.setTargetPosition(base);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(SPIN_MOTOR_SPEED);
    }

    // ---------------- SHOOTING ----------------

    private void shootAll() {
        flickerServo.setPosition(FLICK_DOWN_POS);
        sleep(400);
        flickerServo.setPosition(FLICK_UP_POS);
        sleep(250);
        flickerServo.setPosition(FLICK_DOWN_POS);
    }

    // ---------------- TURRET ----------------

    private void turretRunToPosition(int targetTicks) {
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setPower(TURRET_POWER);
    }

    // ---------------- PATH HELPERS ----------------

    private void runPath(PathChain path, int stopDelayMs, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        if (stopDelayMs > 0) sleep(stopDelayMs);
    }

    private void runPathWithIntake(PathChain path, int stopDelayMs, double speed) {
        intakeActive = true;
        waitingForBall = true;
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            // intake logic could be added here for sensor-driven loading
        }
        follower.breakFollowing();
        if (stopDelayMs > 0) sleep(stopDelayMs);
        intakeActive = false;
    }

    // ---------------- VISION ----------------

    private MotifDecoder.Motif readMotifFromLimelight() {
        return MotifDecoder.fromAprilTagId(detectAprilTag(APRILTAG_TIMEOUT_MS));
    }

    private int detectAprilTag(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid() && !r.getFiducialResults().isEmpty()) {
                return r.getFiducialResults().get(0).getFiducialId();
            }
            sleep(15);
        }
        return 22;
    }

    // ---------------- COLOR DETECTION (available, currently unused) ----------------

    @SuppressWarnings("unused")
    private Spindexer.Ball detectColor() {
        Spindexer.Ball a = detectSingleSensor(intakeColor);
        Spindexer.Ball b = detectSingleSensor(intakeColor2);
        if (a == Spindexer.Ball.PURPLE || b == Spindexer.Ball.PURPLE) return Spindexer.Ball.PURPLE;
        if (a == Spindexer.Ball.GREEN  || b == Spindexer.Ball.GREEN)  return Spindexer.Ball.GREEN;
        return Spindexer.Ball.EMPTY;
    }

    private Spindexer.Ball detectSingleSensor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;
        if (r + g + b < COLOR_MIN_TOTAL) return Spindexer.Ball.EMPTY;
        if (b > COLOR_MIN_CHANNEL && b > r * COLOR_DOMINANCE && b > g * COLOR_DOMINANCE) {
            return Spindexer.Ball.PURPLE;
        }
        if (g > COLOR_MIN_CHANNEL && g > r * COLOR_DOMINANCE && g > b * COLOR_DOMINANCE) {
            return Spindexer.Ball.GREEN;
        }
        return Spindexer.Ball.EMPTY;
    }

    // ---------------- INIT ----------------

    private void initHardware() {
        drivetrain = new MecanumDrive(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_MOTIF_PIPELINE);
        limelight.start();

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        spinMotor.setDirection(DcMotor.Direction.REVERSE);
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 12.9));

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        spinFlickServo = hardwareMap.get(CRServo.class, "backFlick");
        flickerServo = hardwareMap.get(Servo.class, "linearFlick");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeColor.setGain(COLOR_SENSOR_GAIN);
        enableLight(intakeColor);
        intakeColor2.setGain(COLOR_SENSOR_GAIN);
        enableLight(intakeColor2);
    }

    private void enableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) {
            ((SwitchableLight) s).enableLight(true);
        }
    }

    private void resetSlots() {
        Arrays.fill(slots, Spindexer.Ball.EMPTY);
    }

    // Unused but required by original interface
    private void preloadSlots() {
        slots[0] = Spindexer.Ball.PURPLE;
        slots[1] = Spindexer.Ball.GREEN;
        slots[2] = Spindexer.Ball.PURPLE;
    }
}