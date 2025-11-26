package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.redback;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.actual.TeleV2;

@Autonomous(name = "RedBack Auto", group = "Auto")
public class RedBack extends LinearOpMode {

    private Follower follower;
    private GeneratedPathsRedBack paths;


    DcMotor intakeMotor;
    DcMotor launchMotor;
    DcMotor spinMotor;

    Servo pivotServo;
    Servo flickServo;

    NormalizedColorSensor backColor, leftColor, rightColor;

    int[] POSITIONS = {-30, 217, 485}; //{0, 257, 515};
    int[] INTAKE_POSITIONS = {352, -115, 142};

    ballColors[] balls = new ballColors[3];
    int index = 0;
    int currentTarget = 0;

    float gain = 20;
    final float[] hsvValues = new float[3];

    boolean spinControlWas = false;

    enum ballColors {
        PURPLE, GREEN, EMPTY, UNKNOWN
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        if (isStopRequested()) return;

        // Sequence of autonomous (each with a stop + 250ms pause)
        runPath(paths.scan(), 250, 0.75);

        runPath(paths.shoot(), 250, 0.75);
        launchBallAt(0);
        runPath(paths.toIntake1(), 250, 0.75);

        runIntakePath(paths.intake1(), 250, 0.5);

        runPath(paths.shoot2(), 250, 0.75);

        runPath(paths.toIntake2(), 250, 0.75);

        runIntakePath(paths.intake2(), 250, 0.5);

        runPath(paths.shoot3(), 250, 0.75);

        //intakeMotor.setPower(0);

        // End of auto
        telemetry.addLine("RedBack Auto Finished");
        telemetry.update();
    }


    private void moveToPosition(int newIndex, int[] table) {
        currentTarget = table[newIndex];
        runToPosition(spinMotor, currentTarget, 0.4);

    }

    private void runToPosition(DcMotor motor, int targetTicks, double power) {
        motor.setTargetPosition(targetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private void launchBallAt(int index) {
        if (balls[index] != ballColors.EMPTY) {
            flickServo.setPosition(0.6);
            sleep(200);
            flickServo.setPosition(1);

            launchMotor.setPower(1);
            sleep(300);
            launchMotor.setPower(0);

            balls[index] = ballColors.EMPTY;
        }
    }

    private void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // Apply the start pose from GeneratedPathsRedBack
        follower.setPose(GeneratedPathsRedBack.START_POSE);

        //follower = Constants.createFollower(hardwareMap);

        // Set motors to BRAKE to stop drift when idle
        hardwareMap.get(DcMotor.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Load paths
        paths = new GeneratedPathsRedBack(follower);

        telemetry.addLine("Ready to start RedBack Auto");
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

    private void runIntakePath(PathChain path, int stopDelayMs, double speed) {

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
