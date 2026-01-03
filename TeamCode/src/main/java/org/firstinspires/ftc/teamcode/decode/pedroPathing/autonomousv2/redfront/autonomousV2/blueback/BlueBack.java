package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomousv2.redfront.autonomousV2.blueback;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

import java.util.List;

@Autonomous(name = "! Blue Close Auto V2", group = "Auto")
public class BlueBack extends LinearOpMode {

    // -----------------------------
    // HARDWARE
    // -----------------------------
    private Follower follower;
    private GeneratedPathsBlueBack paths;
    private CustomMecanumDrive drivetrain;

    private DcMotor intakeMotor, launchMotor, spinMotor;
    private Servo pivotServo, flickServo;
    private NormalizedColorSensor backColor, leftColor, rightColor;
    private Limelight3A limelight3A;

    // -----------------------------
    // SPINDEXER (NEW SYSTEM)
    // -----------------------------
    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    private static final int[] INTAKE_POS  = {125, 375, 625};
    private static final int[] OUTTAKE_POS = {500, 0, 250};

    private int index = 0;
    private double spinMotorSpeed = 0.35;
    private float gain = 20;

    // -----------------------------
    // BALL PATTERNS
    // -----------------------------
    private final Ball[] pattern21 = {Ball.GREEN, Ball.PURPLE, Ball.PURPLE};
    private final Ball[] pattern22 = {Ball.PURPLE, Ball.GREEN, Ball.PURPLE};
    private final Ball[] pattern23 = {Ball.PURPLE, Ball.PURPLE, Ball.GREEN};

    // -----------------------------
    // RUN OPMODE
    // -----------------------------
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsBlueBack.START_POSE);
        paths = new GeneratedPathsBlueBack(follower);

        telemetry.addLine("BlueBack Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        pivotServo.setPosition(0.57);

        // Initial scan
        scanAllSlots();

        runPath(paths.scan(), 50, 1);

        int tagId = detectAprilTag(1500);
        Ball[] correctPattern =
                tagId == 21 ? pattern21 :
                        tagId == 23 ? pattern23 : pattern22;

        // ---- Shoot preload ----
        runPath(paths.shoot(), 50, 1);
        shootThreeBalls(correctPattern);

        // ---- Intake Ball 3 (NEW CONTINUOUS) ----
        runPath(paths.toIntake1(), 50, 1);
        runIntakePathContinuous(paths.intakeball3(), 0.65);
        scanAllSlots();

        runPath(paths.shoot2(), 50, 1);
        shootThreeBalls(correctPattern);

        // ---- Intake Ball 6 (NEW CONTINUOUS) ----
        runPath(paths.toIntake2(), 250, 0.75);
        runIntakePathContinuous(paths.intakeball6(), 0.65);
        scanAllSlots();

        runPath(paths.shoot3(), 250, 0.75);
        shootThreeBalls(new Ball[]{Ball.PURPLE, Ball.GREEN, Ball.PURPLE});

        telemetry.addLine("BlueBack Auto Finished");
        telemetry.update();
    }

    // -----------------------------
    // PATH HELPERS
    // -----------------------------
    private void runPath(PathChain path, int delayMs, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        follower.breakFollowing();
        stopDriveMotors();
        if (delayMs > 0) sleep(delayMs);
    }

    private void runIntakePathContinuous(PathChain path, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);

        intakeMotor.setPower(1);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            if (!spinMotor.isBusy()) {
                scanAllSlots();
                for (int i = 0; i < 3; i++) {
                    if (slots[i] == Ball.EMPTY) {
                        rotateToIndex(i, true);
                        break;
                    }
                }
            }
        }

        follower.breakFollowing();
        stopDriveMotors();
        intakeMotor.setPower(0);
    }

    private void stopDriveMotors() {
        for (String name : new String[]{"frontLeft","frontRight","backLeft","backRight"}) {
            DcMotor m = hardwareMap.get(DcMotor.class, name);
            m.setPower(0);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // -----------------------------
    // SHOOTING
    // -----------------------------
    private void shootThreeBalls(Ball[] order) {
        launchMotor.setPower(0.9);
        sleep(300);

        for (Ball desired : order) {
            int target = findClosest(desired);
            if (target == -1) continue;

            rotateToIndex(target, false);
            while (opModeIsActive() && spinMotor.isBusy()) idle();

            fire();
            slots[target] = Ball.EMPTY;
        }

        launchMotor.setPower(0);
    }

    private void fire() {
        flickServo.setPosition(0);
        pivotServo.setPosition(0.735);
        sleep(500);

        flickServo.setPosition(0.22);
        sleep(750);

        flickServo.setPosition(0);
        pivotServo.setPosition(0.57);
        sleep(400);
    }

    // -----------------------------
    // SPINDEXER CORE
    // -----------------------------
    private void rotateToIndex(int target, boolean intake) {
        index = target;
        int base = intake ? INTAKE_POS[target] : OUTTAKE_POS[target];
        int pos = closestModular(base, spinMotor.getCurrentPosition());

        spinMotor.setTargetPosition(pos);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinMotorSpeed);
    }

    private int closestModular(int mod, int current) {
        int best = mod, min = Integer.MAX_VALUE;
        for (int k = -2; k <= 2; k++) {
            int c = mod + 750 * k;
            int d = Math.abs(c - current);
            if (d < min) { min = d; best = c; }
        }
        return best;
    }

    private int findClosest(Ball target) {
        for (int i = 0; i < 3; i++) {
            int idx = (index + i) % 3;
            if (slots[idx] == target) return idx;
        }
        return -1;
    }

    // -----------------------------
    // COLOR SENSING
    // -----------------------------
    private void scanAllSlots() {
        slots[0] = detectColor(backColor);
        slots[1] = detectColor(rightColor);
        slots[2] = detectColor(leftColor);
    }

    private Ball detectColor(NormalizedColorSensor s) {
        NormalizedRGBA c = s.getNormalizedColors();
        float r = c.red, g = c.green, b = c.blue;

        if (r + g + b < 0.07f) return Ball.EMPTY;
        if (b > r * 1.35f && b > g * 1.25f && b > 0.12f) return Ball.PURPLE;
        if (g > r * 1.15f && g > b * 1.15f && g > 0.15f) return Ball.GREEN;

        return Ball.EMPTY;
    }

    // -----------------------------
    // APRILTAG
    // -----------------------------
    private int detectAprilTag(long timeout) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeout) {
            LLResult r = limelight3A.getLatestResult();
            if (r != null && r.isValid()) {
                List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
                if (!f.isEmpty()) return f.get(0).getFiducialId();
            }
            sleep(15);
        }
        return 22;
    }

    // -----------------------------
    // HARDWARE INIT
    // -----------------------------
    private void initHardware() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        flickServo = hardwareMap.servo.get("flickServo");
        pivotServo = hardwareMap.servo.get("hoodServo");

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMot\\or.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        backColor = hardwareMap.get(NormalizedColorSensor.class, "backColor");
        leftColor = hardwareMap.get(NormalizedColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(NormalizedColorSensor.class, "rightColor");

        backColor.setGain(gain);
        leftColor.setGain(gain);
        rightColor.setGain(gain);

        if (backColor instanceof SwitchableLight) ((SwitchableLight) backColor).enableLight(true);
        if (leftColor instanceof SwitchableLight) ((SwitchableLight) leftColor).enableLight(true);
        if (rightColor instanceof SwitchableLight) ((SwitchableLight) rightColor).enableLight(true);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(3);
        limelight3A.start();

        drivetrain = new CustomMecanumDrive(hardwareMap);
    }
}
