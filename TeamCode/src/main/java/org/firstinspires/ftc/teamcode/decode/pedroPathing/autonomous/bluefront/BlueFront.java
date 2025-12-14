package org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.bluefront;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.blueback.BlueBack;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.autonomous.bluefront.GeneratedPathsBlueFront;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

import java.util.List;


@Autonomous(name = "! Blue Far Auto", group = "Auto")
public class BlueFront extends LinearOpMode {

    private Follower follower;
    private GeneratedPathsBlueFront paths;
    private CustomMecanumDrive drivetrain;
    DcMotor intakeMotor;
    DcMotor launchMotor;
    DcMotor spinMotor;


    Servo pivotServo;
    Servo flickServo;
    NormalizedColorSensor backColor, leftColor, rightColor;

   // CustomMecanumDrive drivetrain;


    private final int[] POSITIONS = {0, 246, 496};
    private final int[] INTAKE_POSITIONS = {-373, -132, 127}; // {352, -115, 142};


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
        follower.setPose(GeneratedPathsBlueFront.START_POSE);
        paths = new GeneratedPathsBlueFront(follower);


        telemetry.addLine("Ready to start RedFront New Auto");
        telemetry.update();


        waitForStart();


        pivotServo.setPosition(0.6);


        if (isStopRequested()) return;


        // ----------------------
        // 1. Scan all balls
        // ----------------------
        scanAllBalls();
        telemetry.addData("Balls", balls[0] + ", " + balls[1] + ", " + balls[2]);
        telemetry.update();


        intakeMotor.setPower(1);


        //runPath(paths.scan(), 50, 1);


        // SHOULD SCAN APRIL TAG HERE AND DETERMINE CORRECT PATTERN BASED ON TAG ID

        // ------ APRILTAG DETECTION ------
        int tagId = detectAprilTag(2000); // wait up to 1 sec

        if (tagId == 21) {
            correctPattern = pattern21;
        } else if (tagId == 22) {
            correctPattern = pattern22;
        } else if (tagId == 23) {
            correctPattern = pattern23;
        } else {
            correctPattern = pattern22; // fallback
        }


        // ----------------------
        // 2. Move to shooting position
        // ----------------------
        runPath(paths.shoot(), 50, 1);


        // ----------------------
        // 3. Shoot in order: purple → green → purple
        // ----------------------
        shootBallsByColorOrder(correctPattern);
        moveSpindexer(0, INTAKE_POSITIONS);


        // ----------------------
        // 4. Continue auto sequence
        // ----------------------
        runPath(paths.toIntake1(), 50, 0.75);




        runIntakePath(paths.intakeball1(), 50, 0.5);
        intakeMotor.setPower(1);
        sleep(750);
        intakeMotor.setPower(1);
        moveSpindexer(1, INTAKE_POSITIONS);
        sleep(750);


        runIntakePath(paths.intakeball2(), 50, 0.5);
        intakeMotor.setPower(1);
        sleep(750);
        intakeMotor.setPower(1);
        moveSpindexer(2, INTAKE_POSITIONS);
        sleep(750);


        runIntakePath(paths.intakeball3(), 50, 0.5);
        intakeMotor.setPower(1);
        sleep(750);
        intakeMotor.setPower(1);
        moveSpindexer(0, POSITIONS); // moveToPosition
        //moveSpindexer(2, INTAKE_POSITIONS);
        sleep(750);




        scanAllBalls();


        runPath(paths.shoot2(), 50, 0.75);
        scanAllBalls();
        shootBallsByColorOrder(correctPattern);
        moveSpindexer(0, INTAKE_POSITIONS);


        runPath(paths.toIntake2(), 250, 1);


        runIntakePath(paths.intakeball4(), 250, 0.5);


        runIntakePath(paths.intakeball5(), 250, 0.5);


        runIntakePath(paths.intakeball6(), 250, 0.5);


        runPath(paths.shoot3(), 250, 0.75);
        scanAllBalls();
        shootBallsByColorOrder(new ballColors[]{ballColors.PURPLE, ballColors.GREEN, ballColors.PURPLE});


        // End of auto
        telemetry.addLine("RedFront Auto Finished");
        telemetry.update();
    }

    // -----------------------------
    // PATH HELPERS
    // -----------------------------
    private void runPath(PathChain path, int stopDelayMs, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);


        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update();
        }


        follower.breakFollowing();
        stopDriveMotors();


        if (stopDelayMs > 0) sleep(stopDelayMs);
    }


    private void runIntakePath(PathChain path, int stopDelayMs, double speed) {
        follower.setMaxPower(speed);
        follower.followPath(path);


        while (opModeIsActive() && !isStopRequested() && follower.isBusy()) {
            follower.update();
        }


        follower.breakFollowing();
        stopDriveMotors();


        if (stopDelayMs > 0) sleep(stopDelayMs);
    }


    private void stopDriveMotors() {
        String[] driveMotors = {"frontLeft", "frontRight", "backLeft", "backRight"};
        for (String m : driveMotors) {
            DcMotor motor = hardwareMap.get(DcMotor.class, m);
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }


    // -----------------------------
    // BALL SHOOTING HELPERS
    // -----------------------------
    private void shootBallsByColorOrder(ballColors[] order) {

        launchMotor.setPower(1);
        sleep(250);

<<<<<<< Updated upstream
        for (ballColors desired : order) {
=======
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        for (ballColors desired : order) {
=======
        for ( ballColors desired : order) {
>>>>>>> Stashed changes
=======
        for ( ballColors desired : order) {
>>>>>>> Stashed changes
=======
        for ( ballColors desired : order) {
>>>>>>> Stashed changes
>>>>>>> Stashed changes


            int idx = findClosestColor(desired, 0);


            if (balls[idx] == ballColors.EMPTY) continue;


            // move spindexer to that slot
            moveSpindexer(idx, POSITIONS);


            // NEW GANGALANGL wait until sensor confirms correct ball is in the firing chamber ---
            if (waitForBallAtShooter(desired, 1500)) {
                launchBallAt(idx);   // only fires after sensor confirms color
            } else {
                telemetry.addLine("Ball not detected in time, skipping launch");
                telemetry.update();
            }
        }


        launchMotor.setPower(0);
    }
//    private void shootBallsByColorOrder(ballColors[] order) {
//        for (ballColors color : order) {
//            int idx = findClosestColor(color, 0);
//            if (balls[idx] != ballColors.EMPTY) {
//                moveSpindexer(idx, POSITIONS); // moveToPosition
//                // Charge launcher 1 second
// //                launchMotor.setPower(1);
// //                sleep(1000);
// //                launchBallAt(idx);
// //                launchMotor.setPower(0);
// //                sleep(250);
//
//                launchBallAt(idx);
//
//            }
//        }
//    }


    private void launchBallAt(int index) {
        if (balls[index] != ballColors.EMPTY) {


            launchMotor.setPower(1); // 1


            sleep(500);


            flickServo.setPosition(0);
            pivotServo.setPosition(0.735);
            sleep(500);


            flickServo.setPosition(0.22);
            sleep(700);




            flickServo.setPosition(0);
            pivotServo.setPosition(0.6);


            sleep(500);


            balls[index] = ballColors.EMPTY;


        }
    }


    private boolean waitForBallAtShooter(ballColors expected, long timeoutMs) {
        long start = System.currentTimeMillis();


        while (opModeIsActive() && !isStopRequested() &&
                System.currentTimeMillis() - start < timeoutMs) {


            ballColors sensed = detectBallColorFromSensor(backColor);


            if (sensed == expected) {
                return true;
            }


            sleep(20);
        }
        return false;
    }


    // -----------------------------
    // TELEOP METHODS (copied line-for-line)
    // -----------------------------


    private void moveSpindexer(int newIndex, int[] table) {
        pivotServo.setPosition(0.6);
        currentTarget = table[newIndex];
        runToPosition(spinMotor, currentTarget, 0.2);
    }


//    private void moveToPosition(int newIndex, int[] table) {
//        pivotServo.setPosition(0.6);
//        currentTarget = table[newIndex];
//        runToPosition(spinMotor, currentTarget, 0.2);
//    }


    private void runToPosition(DcMotor motor, int targetTicks, double power) {
        motor.setTargetPosition(targetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }


    public int findClosestColor(ballColors target, int currentIndex) {
        for (int offset = 0; offset < balls.length; offset++) {
            int right = (currentIndex + offset) % balls.length;
            int left  = (currentIndex - offset + balls.length) % balls.length;
            if (balls[right] == target) return right;
            if (balls[left] == target)  return left;
        }
        return currentIndex;
    }


    public int findClosestEmpty(int currentIndex) {
        return findClosestColor(ballColors.EMPTY, currentIndex);
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


    // -----------------------------
    // HARDWARE INIT
    // -----------------------------
    private void resetBallArray() {
        balls[0] = ballColors.EMPTY;
        balls[1] = ballColors.EMPTY;
        balls[2] = ballColors.EMPTY;
    }

    private int detectAprilTag(long timeoutMs) {
        long start = System.currentTimeMillis();

        while (opModeIsActive()
                && !isStopRequested()
                && System.currentTimeMillis() - start < timeoutMs) {

            LLResult llResult = limelight3A.getLatestResult();

            if (llResult != null && llResult.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {

                    // Collect valid tag IDs (21, 22, 23)
                    int smallestValid = Integer.MAX_VALUE;

                    for (LLResultTypes.FiducialResult tag : fiducials) {
                        int id = tag.getFiducialId();

                        if (id == 21 || id == 22 || id == 23) {
                            if (id < smallestValid) {
                                smallestValid = id;
                            }
                        }
                    }

                    // If we found at least one valid tag, return the smallest one
                    if (smallestValid != Integer.MAX_VALUE) {
                        telemetry.addData("Using Tag", smallestValid);
                        telemetry.update();
                        return smallestValid;
                    }
                }
            }

            sleep(15);
        }

        telemetry.addLine("NO tag → Defaulting to 22");
        telemetry.update();
        return 22;
    }

    private void initializeHardware() {
        initLauncher();
        initIntake();

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(3);  // APRILTAG PIPELINE
        limelight3A.start();


        backColor  = hardwareMap.get(NormalizedColorSensor.class, "backColor");
        leftColor  = hardwareMap.get(NormalizedColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(NormalizedColorSensor.class, "rightColor");


        backColor.setGain(gain);
        leftColor.setGain(gain);
        rightColor.setGain(gain);


        if (backColor instanceof SwitchableLight)
            ((SwitchableLight) backColor).enableLight(true);
        if (leftColor instanceof SwitchableLight)
            ((SwitchableLight) leftColor).enableLight(true);
        if (rightColor instanceof SwitchableLight)
            ((SwitchableLight) rightColor).enableLight(true);


        drivetrain = new CustomMecanumDrive(hardwareMap);

        pivotServo.setPosition(0.6);

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

    private void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // Apply the start pose from GeneratedPathsRedFront
        follower.setPose(GeneratedPathsBlueFront.START_POSE);

        //follower = Constants.createFollower(hardwareMap);

        // Set motors to BRAKE to stop drift when idle
        hardwareMap.get(DcMotor.class, "frontLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "frontRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backLeft").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotor.class, "backRight").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Load paths
        paths = new GeneratedPathsBlueFront(follower);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Ready to start RedFront Auto");
        telemetry.update();
    }
}
