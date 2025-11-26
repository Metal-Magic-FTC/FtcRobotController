package org.firstinspires.ftc.teamcode.limeLight.pedroPathing.autonomous.bluefront;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.blueback.GeneratedPathsBlueBack;

@Autonomous(name = "BlueBack Auto", group = "Auto")
public class BlueFront extends LinearOpMode {

    private Follower follower;
    private GeneratedPathsBlueBack paths;

    DcMotor intakeMotor;
    Servo leftFlickServo = null;
    Servo rightFlickServo = null;
    Servo middleFlickServo = null;
    Servo leftGate = null;
    Servo rightGate = null;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        if (isStopRequested()) return;

        intakeMotor.setPower(0);

        // Sequence of autonomous (each with a stop + 250ms pause)
        runPath(paths.shoot(), 250, 0.75);

        shoot(0); // eventually replaced by motiff order

        runPath(paths.toIntake1(), 250, 0.75);
        intakeMotor.setPower(1);

        runIntakePath(paths.intake1(), 250, 0.5);
        intakeMotor.setPower(0);

        runPath(paths.shoot2(), 250, 0.75);

        shoot(0); // eventually replaced by motiff order

        runPath(paths.toIntake2(), 250, 0.75);
        intakeMotor.setPower(1);

        runIntakePath(paths.intake2(), 250, 0.5);
        intakeMotor.setPower(0);

        runPath(paths.shoot3(), 250, 0.75);

        shoot(0); // eventually replaced by motiff order

        //intakeMotor.setPower(0);

        // End of auto
        telemetry.addLine("BlueBack Auto Finished");
        telemetry.update();
    }

    private void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // Apply the start pose from GeneratedPathsBlueBack
        follower.setPose(GeneratedPathsBlueBack.START_POSE);

        //follower = Constants.createFollower(hardwareMap);

        // Set motors to BRAKE to stop drift when idle
        hardwareMap.get(DcMotor.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Load paths
        paths = new GeneratedPathsBlueBack(follower);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFlickServo = hardwareMap.servo.get("leftFlickServo");
        middleFlickServo = hardwareMap.servo.get("middleFlickServo");
        rightFlickServo = hardwareMap.servo.get("rightFlickServo");
        leftGate = hardwareMap.servo.get("leftGate");
        rightGate = hardwareMap.servo.get("rightGate");

        gatesClosed();

        telemetry.addLine("Ready to start BlueBack Auto");
        telemetry.update();
    }

    public void gatesClosed() {
        leftGate.setPosition(1); // close
        rightGate.setPosition(0); // close
        leftFlickServo.setPosition(1); // open
        rightFlickServo.setPosition(1); // open
    }

    /**
     * 0 - ppg
     * 1 - pgp
     * 2 - gpp
     * @param order - order of shoot
     */
    public void shoot(int order) {

        if (order == 0) {
            // shoot the middle one
            middleFlickServo.setPosition(0.75); // open
            sleep(400);
            middleFlickServo.setPosition(1);

            sleep(300);

            // move right to middle
            rightGate.setPosition(0.3); // open
            rightFlickServo.setPosition(0.5); // launch
            leftGate.setPosition(1); // close

            sleep(500);
            gatesClosed();

            // shoot middle
            middleFlickServo.setPosition(0.75); // open
            sleep(400);
            middleFlickServo.setPosition(1);

            sleep(300);

            // move left to middle
            leftGate.setPosition(0.7); // open
            leftFlickServo.setPosition(0.4); // launch
            rightGate.setPosition(1); // close

            sleep(500);
            gatesClosed();

            // shoot middle
            middleFlickServo.setPosition(0.75); // open
            sleep(400);
            middleFlickServo.setPosition(1);

            sleep(100);


        }

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
