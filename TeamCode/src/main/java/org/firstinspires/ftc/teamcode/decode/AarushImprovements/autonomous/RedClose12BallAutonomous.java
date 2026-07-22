//package org.firstinspires.ftc.teamcode.decode.AarushImprovements.autonomous;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.SwitchableLight;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//import org.firstinspires.ftc.teamcode.decode.AarushImprovements.paths.RedClose12Paths;
//import org.firstinspires.ftc.teamcode.decode.AarushImprovements.subsystems.MecanumDrive;
//import org.firstinspires.ftc.teamcode.decode.AarushImprovements.subsystems.Spindexer;
//import org.firstinspires.ftc.teamcode.decode.AarushImprovements.util.MotifDecoder;
//import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
//
//import java.util.Arrays;
//
///**
// * Red close 12-ball sorted autonomous (AarushImprovements copy).
// *
// * <p>This is the cleanup of the auto that placed 9th in the Blue Crab division
// * at the Chesapeake Championship. The on-robot behavior is preserved 1:1;
// * cleanup targets readability and reuse only.</p>
// *
// * <h2>Sequence</h2>
// * <ol>
// *     <li>Preload [PURPLE, GREEN, PURPLE], aim at motif, shoot 3.</li>
// *     <li>Intake 3 from human player station, shoot 3.</li>
// *     <li>Intake 3 from far spike mark, shoot 3.</li>
// *     <li>Park in the leave zone for leave points.</li>
// * </ol>
// *
// * <h2>Notes</h2>
// * <ul>
// *     <li>Auto-detects the motif from the obelisk AprilTag (Limelight pipeline 3).</li>
// *     <li>Color sensor gain / thresholds are tuned for the V4 Magic Machine's
// *         REV color sensors — see {@link #detectColor()}.</li>
// *     <li>The turret is driven to a fixed angle for the close-side shoot.</li>
// * </ul>
// */
//@Autonomous(name = "Aarush Red Close 12 Ball", group = "AarushImprovements")
//public class RedClose12BallAutonomous extends LinearOpMode {
//
//    // ---------------- TUNABLE CONSTANTS ----------------
//
//    private static final int[] INTAKE_POS = {125, 375, 625};
//    private static final int[] OUTTAKE_POS = {500, 0, 250};
//
//    private static final double SPIN_MOTOR_SPEED = 0.38;
//    private static final double TURRET_POWER = 0.5;
//    private static final double LAUNCH_VELOCITY = 1800.0;
//    private static final double INTAKE_POWER = -0.6;
//
//    private static final double FLICK_DOWN_POS = 0.96;
//    private static final double FLICK_UP_POS = 0.80;
//    private static final double HOOD_DEFAULT_POS = 0.80;
//
//    /** Per-ball flick: pause after drop, then lift. */
//    private static final long FLICK_LIFT_MS = 240;
//    private static final long FLICK_PAUSE_MS = 250;
//    private static final long FLICK_END_MS = 1_000;
//    private static final long FLICK_FIRST_DROP_MS = 400;
//
//    private static final float COLOR_SENSOR_GAIN = 20.0f;
//    private static final float COLOR_MIN_CHANNEL = 0.06f;
//    private static final float COLOR_DOMINANCE = 1.15f;
//    private static final float COLOR_MIN_TOTAL = 0.08f;
//
//    private static final int TURRET_SHOOT_TICKS = 80;
//    private static final int TURRET_LEAVE_TICKS = 0;
//    private static final int LIMELIGHT_MOTIF_PIPELINE = 3;
//    private static final long APRILTAG_TIMEOUT_MS = 2_000;
//    private static final int SPINDEXER_TICKS_PER_REV = 750;
//
//    /** Pose correction tolerance (inches) — if Limelight sees a tag and pose is off by more than this, correct. */
//    private static final double POSE_CORRECTION_TOLERANCE_IN = 3.0;
//    /** Maximum pose correction per update (inches) — prevents wild jumps. */
//    private static final double MAX_POSE_CORRECTION_IN = 6.0;
//
//    // ---------------- STATE ----------------
//
//    private int spindexerIndex = 0;
//    private boolean intakeActive = false;
//    private boolean waitingForBall = false;
//    private boolean waitingToRotate = false;
//    private long colorDetectedTime = 0;
//    private int nextIndexAfterDelay = -1;
//
//    private final Spindexer.Ball[] slots = {
//            Spindexer.Ball.EMPTY, Spindexer.Ball.EMPTY, Spindexer.Ball.EMPTY
//    };
//
//    // ---------------- HARDWARE ----------------
//
//    private Follower follower;
//    private RedClose12Paths paths;
//    private Limelight3A limelight;
//    private MecanumDrive drivetrain;
//
//    private DcMotor spinMotor;
//    private DcMotorEx launchMotor;
//    private DcMotor intakeMotor;
//    private DcMotor turretMotor;
//    private CRServo spinFlickServo;
//    private Servo flickerServo;
//    private Servo hoodServo;
//    private NormalizedColorSensor intakeColor;
//    private NormalizedColorSensor intakeColor2;
//
//    // ---------------- LIFECYCLE ----------------
//
//    @Override
//    public void runOpMode() {
//        initHardware();
//        preloadSlots();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setPose(RedClose12Paths.START_POSE);
//        paths = new RedClose12Paths(follower);
//
//        hoodServo.setPosition(HOOD_DEFAULT_POS);
//        telemetry.addLine("Ready");
//        telemetry.update();
//
//        waitForStart();
//        spinFlickServo.setPower(1);
//        if (isStopRequested()) return;
//
//        runAutonomous();
//
//        telemetry.addLine("Finished");
//        telemetry.update();
//    }
//
//    // ---------------- SEQUENCE ----------------
//
//    private void runAutonomous() {
//        launchMotor.setVelocity(LAUNCH_VELOCITY);
//        turretRunToPosition(TURRET_SHOOT_TICKS);
//
//        MotifDecoder.Motif motif = readMotifFromLimelight();
//        reportMotif(motif);
//        applyMotifToSpindexer(motif);
//
//        // --- Pose correction from Limelight AprilTag (field-absolute) ---
//        correctPoseFromLimelight();
//
//        runPath(paths.shoot(), 50, 0.8);
//        shootAll();
//
//        startIntakeAt(0);
//
//        // ---- INTAKE 1–3 (human player station) ----
//        runPathWithIntake(paths.toIntake1(), 0, 1.0);
//        runPathWithIntake(paths.intake1(), 0, 0.21);
//        pauseIntaking(500);
//
//        forceLoadSlots(Spindexer.Ball.PURPLE, Spindexer.Ball.PURPLE, Spindexer.Ball.GREEN);
//        applyMotifToSpindexer(motif);
//        runPath(paths.intake1ToShoot2(), 250, 0.5);
//        shootAll();
//
//        startIntakeAt(0);
//
//        // ---- INTAKE 4–6 (far spike mark) ----
//        runPathWithIntake(paths.toIntake2(), 0, 1.0);
//        runPathWithIntake(paths.intake2(), 0, 0.21);
//
//        forceLoadSlots(Spindexer.Ball.PURPLE, Spindexer.Ball.GREEN, Spindexer.Ball.PURPLE);
//        applyMotifToSpindexer(motif);
//        runPath(paths.shoot3(), 250, 0.8);
//        shootAll();
//
//        startIntakeAt(0);
//
//        // ---- LEAVE ----
//        turretRunToPosition(TURRET_LEAVE_TICKS);
//        runPath(paths.leave(), 0, 1.0);
//        resetSlots();
//    }
//
//    // ---------------- HIGH-LEVEL ACTIONS ----------------
//
//    private void preloadSlots() {
//        slots[0] = Spindexer.Ball.PURPLE;
//        slots[1] = Spindexer.Ball.GREEN;
//        slots[2] = Spindexer.Ball.PURPLE;
//    }
//
//    private void startIntakeAt(int index) {
//        intakeMotor.setPower(INTAKE_POWER);
//        intakeActive = true;
//        rotateToIndex(index);
//        resetSlots();
//    }
//
//    private void pauseIntaking(long durationMs) {
//        long start = System.currentTimeMillis();
//        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
//            waitingForBall = true;
//            intakeActive = true;
//            tickIntake();
//        }
//    }
//
//    private void forceLoadSlots(Spindexer.Ball a, Spindexer.Ball b, Spindexer.Ball c) {
//        intakeMotor.setPower(INTAKE_POWER);
//        slots[0] = a;
//        slots[1] = b;
//        slots[2] = c;
//    }
//
//    private void applyMotifToSpindexer(MotifDecoder.Motif motif) {
//        intakeActive = false;
//        int startIdx = MotifDecoder.findBestStartIndex(slots, motif);
//        if (startIdx < 0) startIdx = spindexerIndex;   // safety: stay where we are
//        spindexerIndex = startIdx;
//        rotateToIndex(startIdx);
//    }
//
//    private void reportMotif(MotifDecoder.Motif motif) {
//        telemetry.addData("motif", motif.name());
//        telemetry.addData("slots", "%s %s %s", slots[0], slots[1], slots[2]);
//        telemetry.update();
//    }
//
//    // ---------------- SPINDEXER (POSITION) ----------------
//
//    private void rotateToIndex(int target) {
//        spindexerIndex = target;
//        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
//        int targetPos = closestModular(base, spinMotor.getCurrentPosition());
//        spinMotor.setTargetPosition(targetPos);
//        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        spinMotor.setPower(SPIN_MOTOR_SPEED);
//    }
//
//    private int closestModular(int mod, int current) {
//        int best = mod;
//        int minDiff = Integer.MAX_VALUE;
//        for (int k = -2; k <= 2; k++) {
//            int candidate = mod + SPINDEXER_TICKS_PER_REV * k;
//            int diff = Math.abs(candidate - current);
//            if (diff < minDiff) {
//                minDiff = diff;
//                best = candidate;
//            }
//        }
//        return best;
//    }
//
//    private int findNextEmpty() {
//        for (int i = 0; i < 3; i++) {
//            int idx = (spindexerIndex + i) % 3;
//            if (slots[idx] == Spindexer.Ball.EMPTY) return idx;
//        }
//        return -1;
//    }
//
//    // ---------------- INTAKE LOOP ----------------
//
//    private void tickIntake() {
//        if (waitingForBall && intakeActive && !spinMotor.isBusy() && !waitingToRotate) {
//            Spindexer.Ball detected = detectColor();
//            if (detected != Spindexer.Ball.EMPTY) {
//                slots[spindexerIndex] = detected;
//                waitingForBall = false;
//                nextIndexAfterDelay = findNextEmpty();
//                colorDetectedTime = System.currentTimeMillis();
//                waitingToRotate = true;
//                intakeMotor.setPower(0);
//            }
//        }
//
//        if (waitingToRotate
//                && System.currentTimeMillis() - colorDetectedTime >= FLICK_PAUSE_MS) {
//            if (nextIndexAfterDelay != -1) {
//                rotateToIndex(nextIndexAfterDelay);
//                waitingForBall = true;
//                intakeMotor.setPower(INTAKE_POWER);
//            } else {
//                waitingForBall = true;
//            }
//            waitingToRotate = false;
//        }
//    }
//
//    // ---------------- SHOOTING ----------------
//
//    /**
//     * Launch all 3 balls in slot order, advancing the spindexer 250 ticks
//     * after each shot. After the third ball, reset slots and re-arm intake.
//     */
//    private void shootAll() {
//        intakeMotor.setPower(0);
//        int startPosition = spinMotor.getCurrentPosition();
//
//        spinFlickServo.setPower(1);
//        launchMotor.setVelocity(LAUNCH_VELOCITY);
//        fireOneBall(FLICK_FIRST_DROP_MS);
//
//        moveSpindexerTo(startPosition + 250, SPIN_MOTOR_SPEED);
//        fireOneBall(FLICK_PAUSE_MS);
//
//        moveSpindexerTo(startPosition + 500, SPIN_MOTOR_SPEED);
//        fireOneBall(FLICK_END_MS);
//
//        spinFlickServo.setPower(0);
//        resetSlots();
//        spindexerIndex = (spindexerIndex + 2) % 3;
//        intakeMotor.setPower(INTAKE_POWER);
//    }
//
//    private void fireOneBall(long endPauseMs) {
//        flickerServo.setPosition(FLICK_DOWN_POS);
//        sleep(FLICK_FIRST_DROP_MS);
//        flickerServo.setPosition(FLICK_UP_POS);
//        sleep(FLICK_LIFT_MS);
//        flickerServo.setPosition(FLICK_DOWN_POS);
//        sleep(endPauseMs);
//    }
//
//    private void moveSpindexerTo(int ticks, double power) {
//        spinMotor.setTargetPosition(ticks);
//        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        spinMotor.setPower(power);
//        while (opModeIsActive() && spinMotor.isBusy()) {
//            // balls fire as the spindexer rotates; nothing to do here
//        }
//    }
//
//    // ---------------- TURRET ----------------
//
//    private void turretRunToPosition(int targetTicks) {
//        turretMotor.setTargetPosition(targetTicks);
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turretMotor.setPower(TURRET_POWER);
//    }
//
//    // ---------------- PATH HELPERS ----------------
//
//    private void runPath(PathChain path, int stopDelayMs, double speed) {
//        follower.setMaxPower(speed);
//        follower.followPath(path);
//        while (opModeIsActive() && follower.isBusy()) follower.update();
//        if (stopDelayMs > 0) sleep(stopDelayMs);
//    }
//
//    private void runPathWithIntake(PathChain path, int stopDelayMs, double speed) {
//        intakeActive = true;
//        waitingForBall = true;
//        follower.setMaxPower(speed);
//        follower.followPath(path);
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//            waitingForBall = true;
//            intakeActive = true;
//            tickIntake();
//        }
//        follower.breakFollowing();
//        if (stopDelayMs > 0) sleep(stopDelayMs);
//        intakeActive = false;
//        waitingForBall = false;
//        launchMotor.setVelocity(LAUNCH_VELOCITY);
//    }
//
//    // ---------------- VISION ----------------
//
//    private MotifDecoder.Motif readMotifFromLimelight() {
//        return MotifDecoder.fromAprilTagId(detectAprilTag(APRILTAG_TIMEOUT_MS));
//    }
//
//    private int detectAprilTag(long timeoutMs) {
//        long start = System.currentTimeMillis();
//        while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
//            LLResult r = limelight.getLatestResult();
//            if (r != null && r.isValid() && !r.getFiducialResults().isEmpty()) {
//                return r.getFiducialResults().get(0).getFiducialId();
//            }
//            sleep(15);
//        }
//        return 22; // default motif (PPG)
//    }
//
//    // ---------------- COLOR DETECTION ----------------
//
//    private Spindexer.Ball detectColor() {
//        Spindexer.Ball a = detectSingleSensor(intakeColor);
//        Spindexer.Ball b = detectSingleSensor(intakeColor2);
//        if (a == Spindexer.Ball.PURPLE || b == Spindexer.Ball.PURPLE) return Spindexer.Ball.PURPLE;
//        if (a == Spindexer.Ball.GREEN  || b == Spindexer.Ball.GREEN)  return Spindexer.Ball.GREEN;
//        return Spindexer.Ball.EMPTY;
//    }
//
//    private Spindexer.Ball detectSingleSensor(NormalizedColorSensor sensor) {
//        NormalizedRGBA c = sensor.getNormalizedColors();
//        float r = c.red, g = c.green, b = c.blue;
//        if (r + g + b < COLOR_MIN_TOTAL) return Spindexer.Ball.EMPTY;
//        if (b > COLOR_MIN_CHANNEL && b > r * COLOR_DOMINANCE && b > g * COLOR_DOMINANCE) {
//            return Spindexer.Ball.PURPLE;
//        }
//        if (g > COLOR_MIN_CHANNEL && g > r * COLOR_DOMINANCE && g > b * COLOR_DOMINANCE) {
//            return Spindexer.Ball.GREEN;
//        }
//        return Spindexer.Ball.EMPTY;
//    }
//
//    // ---------------- POSE CORRECTION ----------------
//
//    /**
//     * Reads Limelight AprilTag (ID 20 or 24 — field corner tags) and corrects the
//     * Follower's pose if the difference exceeds tolerance. Uses the same coordinate
//     * conversion as FusedPose.limelightToPedroPathing().
//     */
//    private void correctPoseFromLimelight() {
//        LLResult r = limelight.getLatestResult();
//        if (r == null || !r.isValid()) return;
//        for (LLResultTypes.FiducialResult fid : r.getFiducialResults()) {
//            int id = fid.getFiducialId();
//            if (id != 20 && id != 24) continue; // only corner tags for pose correction
//            Pose3D robotPose3D = fid.getRobotPoseFieldSpace();
//            Pose llPose = new Pose(
//                    robotPose3D.getPosition().x,
//                    robotPose3D.getPosition().y,
//                    robotPose3D.getOrientation().getYaw()
//            );
//            // Convert Limelight pose to Pedro Pathing coordinates (same as FusedPose.limelightToPedroPathing)
//            Pose pedroPose = new Pose(
//                    72 + 39.3701 * llPose.getY(),
//                    72 - 39.3701 * llPose.getX(),
//                    Math.toRadians(llPose.getHeading() - 90)
//            );
//            Pose current = follower.getPose();
//            double dx = pedroPose.getX() - current.getX();
//            double dy = pedroPose.getY() - current.getY();
//            double dist = Math.hypot(dx, dy);
//            if (dist > POSE_CORRECTION_TOLERANCE_IN) {
//                // Cap correction to prevent wild jumps
//                double scale = Math.min(1.0, MAX_POSE_CORRECTION_IN / dist);
//                dx *= scale;
//                dy *= scale;
//                Pose corrected = new Pose(current.getX() + dx, current.getY() + dy, pedroPose.getHeading());
//                follower.setPose(corrected);
//                telemetry.addData("Pose corrected", "dist=%.1f dx=%.1f dy=%.1f", dist, dx, dy);
//                telemetry.update();
//            }
//            break; // only correct from first valid corner tag
//        }
//    }
//
//    // ---------------- INIT ----------------
//
//    private void initHardware() {
//        drivetrain = new MecanumDrive(hardwareMap);
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(LIMELIGHT_MOTIF_PIPELINE);
//        limelight.start();
//
//        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
//        intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
//        intakeColor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeColor2");
//
//        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        spinMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        intakeColor.setGain(COLOR_SENSOR_GAIN);
//        enableLight(intakeColor);
//        intakeColor2.setGain(COLOR_SENSOR_GAIN);
//        enableLight(intakeColor2);
//
//        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
//        launchMotor.setDirection(DcMotorEx.Direction.FORWARD);
//        launchMotor.setPIDFCoefficients(
//                DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(400, 0, 0, 12.9));
//        launchMotor.setVelocity(0);
//
//        hoodServo = hardwareMap.servo.get("hoodServo");
//
//        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        spinFlickServo = hardwareMap.get(CRServo.class, "backFlick");
//        spinFlickServo.setDirection(DcMotorSimple.Direction.FORWARD);
//        spinFlickServo.setPower(0);
//
//        flickerServo = hardwareMap.servo.get("linearFlick");
//        flickerServo.setDirection(Servo.Direction.FORWARD);
//        flickerServo.setPosition(FLICK_DOWN_POS);
//
//        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretMotor.setTargetPosition(0);
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        turretMotor.setPower(0);
//    }
//
//    private void enableLight(NormalizedColorSensor s) {
//        if (s instanceof SwitchableLight) {
//            ((SwitchableLight) s).enableLight(true);
//        }
//    }
//
//    private void resetSlots() {
//        Arrays.fill(slots, Spindexer.Ball.EMPTY);
//    }
//}
