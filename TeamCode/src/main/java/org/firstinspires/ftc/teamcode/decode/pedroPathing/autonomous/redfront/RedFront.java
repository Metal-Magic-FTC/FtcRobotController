package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.redfront;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.redback.GeneratedPathsRedBack;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.redback.RedBackNew;

@Autonomous(name = "Im Gooning to CP", group = "Auto")
public class RedFront extends LinearOpMode {

    private Follower follower;
    private GeneratedPathsRedFront paths;

    DcMotor intakeMotor;
    DcMotor launchMotor;
    DcMotor spinMotor;


    Servo pivotServo;
    Servo flickServo;
    NormalizedColorSensor backColor, leftColor, rightColor;


    int[] POSITIONS = {0, 245, 490}; //{0, 255, 510};
    int[] INTAKE_POSITIONS = {352, -115, 142};


    ballColors[] balls = new ballColors[3];
    int index = 0;
    int currentTarget = 0;


    float gain = 20;


    boolean spinControlWas = false;


    enum ballColors {
        PURPLE, GREEN, EMPTY, UNKNOWN
    }


    ballColors[] pattern21 = new ballColors[]{ballColors.GREEN, ballColors.PURPLE, ballColors.PURPLE};
    ballColors[] pattern22 = new ballColors[]{ballColors.PURPLE, ballColors.GREEN, ballColors.PURPLE};
    ballColors[] pattern23 = new ballColors[]{ballColors.PURPLE, ballColors.PURPLE, ballColors.GREEN};

    private Limelight3A limelight3A;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        resetBallArray();

        ballColors[] correctPattern; // default to 21 at start so no crashing
        // Initialize path follower

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(GeneratedPathsRedBack.START_POSE);
        paths = new GeneratedPathsRedFront(follower);

        telemetry.addLine("Ready to start RedBack New Auto");
        telemetry.update();


        waitForStart();


        pivotServo.setPosition(0.6);

        scanAllBalls();
        telemetry.addData("Balls", balls[0] + ", " + balls[1] + ", " + balls[2]);
        telemetry.update();

        if (isStopRequested()) return;

        intakeMotor.setPower(0);

        runPath(paths.scan(), 250, 1);

        // Sequence of autonomous (each with a stop + 250ms pause)
        runPath(paths.shoot(), 250, 0.75);

        runPath(paths.toIntake1(), 250, 0.75);
        intakeMotor.setPower(1);

        runPath(paths.intake1(), 250, 0.5);
        intakeMotor.setPower(0);

        runPath(paths.shoot2(), 250, 0.75);

        runPath(paths.toIntake2(), 250, 0.75);
        intakeMotor.setPower(1);

        runPath(paths.intake2(), 250, 0.5);
        intakeMotor.setPower(0);

        runPath(paths.shoot3(), 250, 0.75);

        //intakeMotor.setPower(0);

        // End of auto
        telemetry.addLine("RedFront Auto Finished");
        telemetry.update();
    }
    private void initializeHardware() {
        initLauncher();
        initIntake();
    }

    private void initLauncher() {
        pivotServo = hardwareMap.servo.get("launchServo");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        flickServo = hardwareMap.servo.get("flickServo");
    }
    private void initIntake() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.REVERSE);
    }


    private void resetBallArray() {
        balls[0] = ballColors.EMPTY;
        balls[1] = ballColors.EMPTY;
        balls[2] = ballColors.EMPTY;
    }

    public void scanAllBalls() {
        balls[0] = detectBallColorFromSensor(backColor);
        balls[1] = detectBallColorFromSensor(rightColor);
        balls[2] = detectBallColorFromSensor(leftColor);
    }
    private ballColors detectBallColorFromSensor(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();


        float r = c.red, g = c.green, b = c.blue;
        float tol = 0.20f;


        if (b > r * (1 + tol) && b > g * (1 + tol)) return ballColors.PURPLE;
        if (g > r * (1 + tol) && g > b * (1 + tol)) return ballColors.GREEN;


        if (r > 0.01 || g > 0.01 || b > 0.01) return ballColors.UNKNOWN;
        return ballColors.EMPTY;
    }

    private void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // Apply the start pose from GeneratedPathsRedFront
        follower.setPose(GeneratedPathsRedFront.START_POSE);

        //follower = Constants.createFollower(hardwareMap);

        // Set motors to BRAKE to stop drift when idle
        hardwareMap.get(DcMotor.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Load paths
        paths = new GeneratedPathsRedFront(follower);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Ready to start RedFront Auto");
        telemetry.update();
    }

    private void runPath(PathChain path, int stopDelayMs, double speed) {

        follower.setMaxPower(speed); // 0.6 of normal power

        follower.followPath(path);

        // run until path done
        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update();
        }


        follower.breakFollowing();

        // Fully stop all drive motors
        hardwareMap.get(DcMotor.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontLeft").setPower(0);
        hardwareMap.get(DcMotor.class, "frontRight").setPower(0);
        hardwareMap.get(DcMotor.class, "backLeft").setPower(0);
        hardwareMap.get(DcMotor.class, "backRight").setPower(0);

        if (stopDelayMs > 0) sleep(stopDelayMs);
    }

}
