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

import org.firstinspires.ftc.teamcode.decode.AarushImprovements.paths.BlueFar12Paths;
import org.firstinspires.ftc.teamcode.decode.AarushImprovements.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.AarushImprovements.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.decode.AarushImprovements.util.MotifDecoder;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

import java.util.Arrays;

/**
 * Blue far 12-ball sorted autonomous for the DECODE game (AarushImprovements copy).
 * 
 * <p>Starts at the far wall (opposite the Blue alliance station) and traverses the full field.
 * Mirrors the Red far auto across the X-axis.</p>
 */
@Autonomous(name = "Aarush Blue Far 12 Ball", group = "AarushImprovements")
public class BlueFar12BallAutonomous extends LinearOpMode {

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

    private static final long FLICK_LIFT_MS = 240;
    private static final long FLICK_PAUSE_MS = 250;
    private static final long FLICK_END_MS = 1000;
    private static final long FLICK_FIRST_DROP_MS = 400;

    private static final float COLOR_SENSOR_GAIN = 20.0f;
    private static final float COLOR_MIN_CHANNEL = 0.06f;
    private static final float COLOR_DOMINANCE = 1.15f;
    private static final float COLOR_MIN_TOTAL = 0.08f;

    private static final int TURRET_SHOOT_TICKS = -76;   // flipped for Blue
    private static final int TURRET_LEAVE_TICKS = 0;
    private static final int LIMELIGHT_MOTIF_PIPELINE = 3;
    private static final long APRILTAG_TIMEOUT_MS = 2000;

    // ---------------- STATE ----------------

    private int spindexerIndex = 0;
    private boolean intakeActive = false;
    private boolean waitingForBall = false;

    private final Spindexer.Ball[] slots = {
            Spindexer.Ball.EMPTY, Spindexer.Ball.EMPTY, Spindexer.Ball.EMPTY
    };

    // ---------------- HARDWARE ----------------

    private Follower follower;
    private BlueFar12Paths paths;
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
        follower.setPose(BlueFar12Paths.START_POSE);
        paths = new BlueFar12Paths(follower);

        hoodServo.setPosition(HOOD_DEFAULT_POS);
        telemetry.addLine("Ready - Blue Far 12 Ball");
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
        reportMotif(motif);
        applyMotifToSpindexer(motif);

        runPath(paths.scan(), 0, 0.9);
        runPath(paths.shoot(), 50, 0.8);
        shootAll();

        startIntakeAt(0);

        // ---- INTAKE 1-3 (human player station) ----
        runPathWithIntake(paths.toIntake1(), 0, 1.0);
        runPathWithIntake(paths.intake1(), 0, 0.21);
        pauseIntaking(500);

        forceLoadSlots(Spindexer.Ball.PURPLE, Spindexer.Ball.PURPLE, Spindexer.Ball.GREEN);
        applyMotifToSpindexer(motif);
        runPath(paths.gate(), 750, 0.5);
        runPath(paths.shoot2(), 250, 0.5);
        shootAll();

        startIntakeAt(0);

        // ---- INTAKE 4-6 (far spike mark) ----
        runPathWithIntake(paths.toIntake2(), 0, 1.0);
        runPathWithIntake(paths.intake2(), 0, 0.21);

        forceLoadSlots(Spindexer.Ball.PURPLE, Spindexer.Ball.GREEN, Spindexer.Ball.PURPLE);
        applyMotifToSpindexer(motif);
        runPath(paths.shoot3(), 250, 0.8);
        shootAll();

        startIntakeAt(0);

        // ---- INTAKE 7-9 (far spike mark continued) ----
        runPathWithIntake(paths.toIntake3(), 0, 1.0);
        runPathWithIntake(paths.intake3(), 0, 0.21);

        forceLoadSlots(Spindexer.Ball.GREEN, Spindexer.Ball.PURPLE, Spindexer.Ball.PURPLE);
        applyMotifToSpindexer(motif);
        runPath(paths.shoot4(), 250, 0.8);
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

    private void pauseIntaking(long durationMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            waitingForBall = true;
            intakeActive = true;
            tickIntake();
        }
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

    private void reportMotif(MotifDecoder.Motif motif) {
        telemetry.addData("motif", motif.name());
        telemetry.addData("slots", "%s %s %s", slots[0], slots[1], slots[2]);
        telemetry.update();
    }

    // ---------------- SPINDEXER ----------------

    private void rotateToIndex(int target) {
        spindexerIndex = target;
        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
        spinMotor.setTargetPosition(base);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.38);
    }

    private int findNextEmpty() {
        for (int i = 0; i < 3; i++) {
            int idx = (spindexerIndex + i) % 3;
            if (slots[idx] == Spindexer.Ball.EMPTY) return idx;
        }
        return -1;
    }

    // ---------------- INTAKE LOOP ----------------

    private void tickIntake() {
        if (waitingForBall && intakeActive && !spinMotor.isBusy()) {
            Spindexer.Ball detected = detectColor();
            if (detected != Spindexer.Ball.EMPTY) {
                slots[spindexerIndex] = detected;
                waitingForBall = false;
                int nextEmpty = findNextEmpty();
                if (nextEmpty != -1) {
                    rotateToIndex(nextEmpty);
                    waitingForBall = true;
                    intakeMotor.setPower(INTAKE_POWER);
                } else {
                    waitingForBall = true;
                }
            }
        }
    }

    // ---------------- SHOOTING ----------------

    private void shootAll() {
        intakeMotor.setPower(0);
        int startPosition = spinMotor.getCurrentPosition();

        spinFlickServo.setPower(1);
        launchMotor.setVelocity(LAUNCH_VELOCITY);
        flickerServo.setPosition(FLICK_DOWN_POS);
        sleep(400);
        flickerServo.setPosition(FLICK_UP_POS);
        sleep(250);
        flickerServo.setPosition(FLICK_DOWN_POS);
        sleep(1000);

        spinFlickServo.setPower(0);
        resetSlots();
        spindexerIndex = (spindexerIndex + 2) % 3;
        intakeMotor.setPower(INTAKE_POWER);
    }

    // ---------------- TURRET ----------------

    private void turretRunToPosition(int targetTicks) {
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.5);
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
            tickIntake();
        }
        follower.breakFollowing();
        intakeActive = false;
    }

    // ---------------- VISION ----------------

    private MotifDecoder.Motif readMotifFromLimelight() {
        return MotifDecoder.fromAprilTagId(detectAprilTag(2000));
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

    // ---------------- COLOR DETECTION ----------------

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
        if (r + g + b < 0.08f) return Spindexer.Ball.EMPTY;
        if (b > 0.06f && b > r * 1.15f && b > g * 1.15f) return Spindexer.Ball.PURPLE;
        if (g > 0.06f && g > r * 1.15f && g > b * 1.15f) return Spindexer.Ball.GREEN;
        return Spindexer.Ball.EMPTY;
    }

    // ---------------- INIT ----------------

    private void initHardware() {
        drivetrain = new MecanumDrive(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
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

        intakeColor.setGain(20.0f);
        enableLight(intakeColor);
        intakeColor2.setGain(20.0f);
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
}