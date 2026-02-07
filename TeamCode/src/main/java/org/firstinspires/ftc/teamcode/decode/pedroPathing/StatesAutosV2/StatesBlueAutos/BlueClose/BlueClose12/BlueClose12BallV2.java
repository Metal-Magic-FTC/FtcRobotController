package org.firstinspires.ftc.teamcode.decode.pedroPathing.StatesAutosV2.StatesBlueAutos.BlueClose.BlueClose12;

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
import org.firstinspires.ftc.teamcode.decode.pedroPathing.StatesAutosV2.StatesBlueAutos.BlueClose.BlueClosePaths;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.StatesAutosV2.StatesBlueAutos.BlueClose.GeneratedPathsBlue12BallCloseV2;

import java.util.Arrays;

//@Disabled
@Autonomous(name = "!!!!! LEBRON States Blue Close 12 Ball")
public class BlueClose12BallV2 extends LinearOpMode {

    private int index = 0;
    private float gain = 20;

    private enum Ball { EMPTY, PURPLE, GREEN }
    private Ball[] slots = {Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};

    // ---------------- DRIVE ----------------
    private Follower follower;
    private GeneratedPathsBlue12BallCloseV2 paths;
    private Limelight3A limelight;

    // ---------------- HARDWARE ----------------
    private DcMotor spinMotor;
    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;
    private DcMotor turretMotor;

    private CRServo spinFlickServo;
    private Servo flickerServo;
    private Servo hoodServo;

    private NormalizedColorSensor intakeColor;
    private NormalizedColorSensor intakeColor2;

    private static final int[] OUTTAKE_POS = {500, 0, 250};
    private static final int[] INTAKE_POS  = {125, 375, 625};

    private double spinMotorSpeed = 0.38;
    private boolean intakeActive = false;
    private boolean waitingToRotate = false;
    private boolean waitingForBall = false;
    private long colorDetectedTime = 0;
    private static final long COLOR_DELAY_MS = 2;
    private int nextIndexAfterDelay = -1;

    private int lastSpinTarget = 0;

    private final double flickPositionUp = 0.80;
    private final double flickPositionDown = 0.96;

    private double turretPower = 0.5;

    // ---------------- RUN ----------------
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        resetSlots();

        slots[0] = Ball.PURPLE;
        slots[1] = Ball.GREEN;
        slots[2] = Ball.PURPLE;

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsBlue12BallCloseV2.START_POSE);
        paths = new GeneratedPathsBlue12BallCloseV2(follower);

        hoodServo.setPosition(0.80);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        launchMotor.setVelocity(1800);

        turretRunToPosition(-76); // 🔵 FLIPPED FOR BLUE

        Ball[] pattern = getPatternFromTag();
        aimToPattern(pattern);

        runPath(paths.shoot(), 50, 0.8);
        shootAll();

        intakeMotor.setPower(-0.6);
        intakeActive = true;
        rotateToIndex(0);
        resetSlots();

        runPathWithIntake(paths.toIntake1(), 0, 1);
        runPathWithIntake(paths.intake1(), 0, 0.21);

        intakeMotor.setPower(-0.6);
        slots[0] = Ball.PURPLE;
        slots[1] = Ball.PURPLE;
        slots[2] = Ball.GREEN;

        aimToPattern(pattern);
        runPath(paths.shoot2(), 0, 0.5);
        shootAll();

        intakeMotor.setPower(-0.6);
        rotateToIndex(0);
        resetSlots();

        runPathWithIntake(paths.toIntake2(), 0, 1);
        runPathWithIntake(paths.intake2(), 0, 0.21);

        intakeMotor.setPower(-0.6);
        slots[0] = Ball.PURPLE;
        slots[1] = Ball.GREEN;
        slots[2] = Ball.PURPLE;

        aimToPattern(pattern);
        runPath(paths.shoot3(), 0, 0.8);
        shootAll();

        turretRunToPosition(0);
        runPath(paths.leave(), 0, 1);

        resetSlots();
        telemetry.addLine("Finished");
        telemetry.update();
    }

    // ---------------- PATH HELPERS ----------------
    private void runPath(PathChain path, int stopDelay, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        if (stopDelay > 0) sleep(stopDelay);
    }

    private void runPathWithIntake(PathChain path, int stopDelay, double speed) {
        intakeActive = true;
        waitingForBall = true;

        follower.setMaxPower(speed);
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            intake();
        }
        follower.breakFollowing();
        if (stopDelay > 0) sleep(stopDelay);
        intakeActive = false;
    }

    // ---------------- APRILTAG ----------------
    private Ball[] getPatternFromTag() {
        int id = detectAprilTag(2000);
        if (id == 21) return new Ball[]{Ball.GREEN, Ball.PURPLE, Ball.PURPLE};
        if (id == 23) return new Ball[]{Ball.PURPLE, Ball.PURPLE, Ball.GREEN};
        return new Ball[]{Ball.PURPLE, Ball.GREEN, Ball.PURPLE};
    }

    private int detectAprilTag(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < timeoutMs) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid() && !r.getFiducialResults().isEmpty())
                return r.getFiducialResults().get(0).getFiducialId();
            sleep(15);
        }
        return 22;
    }

    // ---------------- INIT ----------------
    private void initHardware() {

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
                new PIDFCoefficients(300, 0, 0, 12.9)
        );

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        spinFlickServo = hardwareMap.get(CRServo.class, "backFlick");
        flickerServo = hardwareMap.get(Servo.class, "linearFlick");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE); // 🔵 FLIPPED
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void resetSlots() {
        for (int i = 0; i < 3; i++) slots[i] = Ball.EMPTY;
    }

    private void rotateToIndex(int target) {
        index = target;
        int base = intakeActive ? INTAKE_POS[target] : OUTTAKE_POS[target];
        spinMotor.setTargetPosition(base);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinMotorSpeed);
    }

    private void aimToPattern(Ball[] pattern) {
        rotateToIndex(findBestStartIndex(pattern));
    }

    private int findBestStartIndex(Ball[] pattern) {
        int greenIndex = Arrays.asList(slots).indexOf(Ball.GREEN);
        if (greenIndex == -1) return 0;
        if (pattern[0] == Ball.GREEN) return greenIndex;
        if (pattern[1] == Ball.GREEN) return (greenIndex + 2) % 3;
        return (greenIndex + 1) % 3;
    }

    private void intake() {}

    private void shootAll() {
        flickerServo.setPosition(flickPositionDown);
        sleep(400);
        flickerServo.setPosition(flickPositionUp);
        sleep(250);
        flickerServo.setPosition(flickPositionDown);
    }

    private void turretRunToPosition(int targetTicks) {
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setPower(turretPower);
    }
}
