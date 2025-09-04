package org.firstinspires.ftc.teamcode.mmintothedeep.Autonomous.Tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.mmintothedeep.UtilityValues;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;


import java.util.Date;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Blue TEST ONLY: LEFT of Gate", group="Autonomous")
@Disabled
public class BlueAutoLeftTest extends LinearOpMode {
    Date currentTime = new Date();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public Servo gripperServo1 = null;
    public Servo pivotServo = null;
    public DcMotor linearSlideMotor = null;
    public DcMotor linearActuatorMotor = null;

    CRServo armMotor = null;
    static final double MOTOR_TICK_COUNTS = UtilityValues.motorTicks; // goBILDA 5203 series Yellow Jacket
    // figure out how many times we need to turn the wheels to go a certain distance
    // the distance you drive with one turn of the wheel is the circumference of the wheel
    // The wheel's Diameter is 96mm. To convert mm to inches, divide by 25.4
    static final double WHEEL_DIAMETER_INCHES = UtilityValues.wheelDiameter / 25.4; // in Inches
    static final double CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES; // pi * the diameter of the wheels in inches

    static final double DEGREES_MOTOR_MOVES_IN_1_REV = 45.0;

    static final double SPEED = UtilityValues.SPEED; // Motor Power setting

    VisionPortal visionPortal;
    VisionPortal visionPortal2;
    VisionPortal visionPortal3;
    AprilTagProcessor tagProcessor;
    AprilTagProcessor tagProcessor2;
    ColorBlobLocatorProcessor colorLocator;
    private int myExposure;
    private int myGain;

    @Override
    public void runOpMode() {

        initPortal();
        initMotor();
        getCameraSetting();
        myExposure = 26;
        myGain = 255;
        setManualExposure(myExposure, myGain);
        waitForStart();


      /*
        ============================
        THIS IS THE ACTUAL DRIVING
        ============================
       */

       /*
        METAL MAGIC INTO THE DEEP
        THIS CODE STARTS ON THE LEFT SIDE OF THE BLUE SIDE (closer to backdrop)
        STACKS PIXEL AND PARKS IN CORNER
        THIS IS A TEST FILE TO TEST AUTONOMOUS CODE TO BE EVENTUALLY USED
        */
        //sleep lines are to avoid two lines of codes running at the same time
//        pivotServo.setPosition(0.6);
//        gripperServo1.setPosition(0);
        moveStraightLine(12); //33
        strafe(20);
//        linearSlideMovement(800, false);
//        strafeDiagonalRight(30);
//        moveStraightLine(2.5);
//        //moveStraightLine(-1);
//        pivotServo.setPosition(0.635);
//        linearSlideMovement(300, true);
//        sleep(500);
//        gripperServo1.setPosition(0.3);
//        sleep(600);
//        moveStraightLine(-10);
//        linearSlideMovement(50, false);
//        rotate(-90);
//        sleep(1000);
//        //moveStraightLine(30);
//        moveStraightLine(20);
//        tagTelemetry(1);
//        sleep(1000);
//        align(0, 16, 0, 1);
//        strafe(24);
//        pivotServo.setPosition(0.9);
//        gripperServo1.setPosition(0.1);
        alignSample();
        pivotServo.setPosition(0.9);
        gripperServo1.setPosition(0.1);
        //rotate(90);
        //strafe(24);


        /*sleep(200);
        moveStraightLine(3);
        sleep(200);
        moveLinearSlide(26); //to 26 inches high
        sleep(200);
        gripperServo1.setPosition(0.3);
        sleep(200);
        moveLinearSlide(-24);
        sleep(200);
        moveStraightLine(-3);
        sleep(200);
        rotate(-90);
        align(0, 24, 90, 1);
        pivotServo.setPosition(1);
        gripperServo1.setPosition(0);
        rotate(-90);
        alignToDefault("basket", 1);
        moveLinearSlide(43.5);
        moveStraightLine(2);
        gripperServo1.setPosition(0.3);*/


        /*strafeDiagonalLeft(5);
        alignToDefault("chamber", 2);
        rotate(-90);
        moveLinearSlide(27);
        gripperServo1.setPosition(0.3);

        moveLinearSlide(0); //or 3 because 3 or less defaults to 0

        align(0, 16, 0, 1);
        alignToDefault("basket", 1);
        moveLinearSlide(43);
        moveLinearSlide(0);

        returnBackTo13Basket(); //returning to apriltag 13 for scanning
        //in place of color scanning to test timing
        //block 1
        align(24, 24, 0, 1);
        strafe(-24);
        alignToDefault("basket", 1);
        moveLinearSlide(43);
        moveLinearSlide(0);
        //block 2
        align(24, 12, 0, 1);
        strafe(-24);
        moveStraightLine(-12);
        alignToDefault("basket", 1);
        moveLinearSlide(43);
        moveLinearSlide(0);
        //block 3
        align(24, 5, 0, 1);
        strafe(-24);
        moveStraightLine(-19);
        alignToDefault("basket", 1);
        moveLinearSlide(43);
        moveLinearSlide(0);*/

        //Termination
        if (currentTime.getTime() > 20000) {
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }

    }

    public void alignToOffset(double x, double y, double dir, int vision) {

        double offset;
        if (vision == 1) {
            offset = UtilityValues.offsetCamera1;
            alignRotate(0, vision);
            alignX(x+offset, vision);
            alignY(y, vision);
            rotate(dir);
        }

    }

    public void alignToDefaultOffset(String s, int vision) {
        if (vision == 1) {
            if (Objects.equals(s, "basket")) {
                if (tagProcessor.getDetections().get(0).id == 11) {
                    alignToOffset(0, 70, 180, vision);
                    alignToOffset(0, 16, -45, vision); //now with tag 13
                } else if (tagProcessor.getDetections().get(0).id == 12) {
                    alignToOffset(-50, 16, 90, vision);
                    alignToOffset(0, 16, -45, vision); //now with tag 13
                } else if (tagProcessor.getDetections().get(0).id == 13) {
                    alignToOffset(0, 16, -45, vision);
                }
            }

            if (Objects.equals(s, "chamber")) {
            }
        } else if (vision == 2) {
            if (Objects.equals(s, "chamber")) {
                if (tagProcessor2.getDetections().get(0).id == 12) {
                    pivotServo.setPosition(0.6);
                    gripperServo1.setPosition(0);
                    alignY(24, vision);
                    linearSlideMovement(1300, false);
                    strafeDiagonalLeft(15);
                    //moveStraightLine(-1);
                    pivotServo.setPosition(0.635);
                    linearSlideMovement(300, true);
                    gripperServo1.setPosition(0.3);
                }
            }
        }
    }

    public void linearSlideMovement(double y, boolean maxPower) {
        double up;
        if (y > linearSlideMotor.getCurrentPosition()) {
            while (linearSlideMotor.getCurrentPosition() < y) {
                up = Math.sin(((double) (4000 - linearSlideMotor.getCurrentPosition()) / 4000) * Math.PI / 2);
                if (maxPower) {
                    linearSlideMotor.setPower(1);
                } else {
                    linearSlideMotor.setPower(up);
                }
            }
            linearSlideMotor.setPower(0);
        } else {
            while (linearSlideMotor.getCurrentPosition() > y) {
                up = Math.sin(((double) (1000+linearSlideMotor.getCurrentPosition()) /4000)*Math.PI/2);
                if (maxPower) {
                    linearSlideMotor.setPower(-1);
                } else {
                    linearSlideMotor.setPower(-1*up);
                }

            }
            linearSlideMotor.setPower(0);
        }
    }

    public void returnBackTo13Basket() {
        rotate(45);
        strafe(0);
    }

    public void alignToDefault(String s, int vision) {
        if (vision == 1) {
            if (Objects.equals(s, "basket")) {
                if (tagProcessor.getDetections().get(0).id == 11) {
                    align(0, 70, 180, vision);
                    align(0, 16, -45, vision); //now with tag 13
                } else if (tagProcessor.getDetections().get(0).id == 12) {
                    align(-50, 16, 90, vision);
                    align(0, 16, -45, vision); //now with tag 13
                } else if (tagProcessor.getDetections().get(0).id == 13) {
                    align(0, 16, -45, vision);
                }
            }

            if (Objects.equals(s, "chamber")) {
            }
        } else if (vision == 2) {
            if (Objects.equals(s, "chamber")) {
                if (tagProcessor2.getDetections().get(0).id == 12) {
                    align(0, 16, 0, 2);
                    moveStraightLine(5);
                }
            }
        }
    }

    public void alignTo(String s, int tagID, int vision) {

        if (Objects.equals(s, "basket")) {
            if (tagID == 12) {
                align(55, 16, 45, vision);
            }

        }

        if (Objects.equals(s, "chamber")) {
            if (tagID == 12) {
                align(0, 26, 180, vision);
            }
        }

        if (tagID == 13) {
            if (Objects.equals(s, "basket")) {
                alignRotate(0, vision);
                alignY(16, vision);
                alignX(-16, vision);
                alignRotate(-45, vision);

            }
        }

    }

    public void initMotor() {
        /* Assign all the motors */
        //drivetrain
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

        //claw
        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        pivotServo = hardwareMap.servo.get("pivotServo");

        linearSlideMotor = hardwareMap.dcMotor.get("hangSlideMotor");
        linearActuatorMotor = hardwareMap.dcMotor.get("linearActuatorMotor");

        // Set all the right motor directions
        leftFrontDrive.setDirection(UtilityValues.finalLeftFrontDirection);
        leftBackDrive.setDirection(UtilityValues.finalLeftBackDirection);
        rightFrontDrive.setDirection(UtilityValues.finalRightFrontDirection);
        rightBackDrive.setDirection(UtilityValues.finalRightBackDirection);


        // Reset encoders positions
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotor.setDirection(CRServo.Direction.FORWARD);

        /*while (hangSlideMotor.getCurrentPosition() > 0) {
            hangSlideMotor.setPower(-0.5);
        }
        while (hangSlideMotor.getCurrentPosition() < 0) {
            hangSlideMotor.setPower(0.3);
        }*/

        ((ServoImplEx) pivotServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripperServo1.setPosition(0);
        pivotServo.setPosition(0);
        gripperServo1.setPosition(0);
        pivotServo.setPosition(0);

        // ABOVE THIS, THE ENCODERS AND MOTOR ARE NOW RESET

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripperServo1.setPosition(0);
        pivotServo.setPosition(0.48);

        linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initPortal() {

        // Because we want to show two camera feeds simultaneously, we need to inform
        // the SDK that we want it to split the camera monitor area into two smaller
        // areas for us. It will then give us View IDs which we can pass to the individual
        // vision portals to allow them to properly hook into the UI in tandem.
        int[] viewIds = VisionPortal.makeMultiPortalView(3, VisionPortal.MultiPortalLayout.VERTICAL);

        // We extract the two view IDs from the array to make our lives a little easier later.
        // NB: the array is 2 long because we asked for 2 portals up above.
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];
        int portal3ViewId = viewIds[2];

        //drawing information on the driver station camera screen
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(484.149, 484.149, 309.846, 272.681)
                .build();

        tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(513.474, 513.474, 316.919, 249.760)
                .build();

        //stating the webcam
        visionPortal = new VisionPortal.Builder()
                .setLiveViewContainerId(portal1ViewId)
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .setCameraResolution(new Size(640, 480))
                .build();

        visionPortal2 = new VisionPortal.Builder()
                .setLiveViewContainerId(portal2ViewId)
                .addProcessor(tagProcessor2)
                .setCamera(hardwareMap.get(WebcamName.class, "diddyCam"))
                .setCameraResolution(new Size(640, 480))
                .build();

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0, 0.5, -1))  // search central 1/4 of camera view
                // .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                //.setErodeSize(6)
                //.setDilateSize(6)
                .build();

        visionPortal3 = new VisionPortal.Builder()
                .setLiveViewContainerId(portal3ViewId)
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "testWebcam"))
                .build();

    }

    public void tagTelemetry(int vision) {
        telemetry.addData("Vision portal: ", vision);
        if (vision == 1) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                //sending telemetry values to the driver station
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("id", tag.id);
            }
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor2.getDetections().get(0);
                //sending telemetry values to the driver station
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("id", tag.id);
            }
        }
        telemetry.update();
    }

    public void align(int x, int y, int dir, int vision) {
        alignRotate(0, vision);
        alignY(y, vision);
        alignX(x, vision);
        rotate(dir);
    }

    public void alignRotate(int dir, int vision) {

        double rotateNew;
        double originalY;
        double rotateRadians;
        double correctX;

        if (vision == 1) {
            if (tagProcessor.getDetections().size() > 0) {
                rotateNew = tagProcessor.getDetections().get(0).ftcPose.yaw - dir;
                originalY = tagProcessor.getDetections().get(0).ftcPose.y;

                if (tagProcessor.getDetections().get(0).ftcPose.yaw < (-0.5 + dir)) { //0.5 is buffer
                    //strafe(1);
                    rotate(-rotateNew);
                }
                else if (tagProcessor.getDetections().get(0).ftcPose.yaw > (0.5 + dir)) { //0.5 is buffer
                    //strafe(-1);
                    rotate(-rotateNew);
                }

                rotateRadians = Math.toRadians(rotateNew);
                correctX = Math.tan(rotateRadians) * originalY;
                strafe(correctX);

            }
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                rotateNew = tagProcessor2.getDetections().get(0).ftcPose.yaw - dir;
                originalY = tagProcessor2.getDetections().get(0).ftcPose.y;

                if (tagProcessor2.getDetections().get(0).ftcPose.yaw < (-0.5 + dir)) { //0.5 is buffer
                    //strafe(1);
                    rotate(-rotateNew);
                }
                if (tagProcessor2.getDetections().get(0).ftcPose.yaw > (0.5 + dir)) { //0.5 is buffer
                    //strafe(-1);
                    rotate(-rotateNew);
                }

                rotateRadians = Math.toRadians(rotateNew);
                correctX = Math.tan(rotateRadians) * originalY;
                strafe(-1*correctX);
            }
        }

    }

    public void alignX(double x, int vision) {

        double xPosNew;
        //alignX(-1, 1, 12);
        if (vision == 1) {
            if (tagProcessor.getDetections().size() > 0) {
                xPosNew = tagProcessor.getDetections().get(0).ftcPose.x - x;

                if (tagProcessor.getDetections().get(0).ftcPose.x < (-0.5 + x)) { //0.5 is buffer
                    //strafe(1);
                    strafe(1 * xPosNew);
                }
                if (tagProcessor.getDetections().get(0).ftcPose.x > (0.5 + x)) { //0.5 is buffer
                    //strafe(-1);
                    strafe(1 * xPosNew);
                }
            }
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                xPosNew = tagProcessor2.getDetections().get(0).ftcPose.x - x;

                if (tagProcessor2.getDetections().get(0).ftcPose.x < (-0.5 + x)) { //0.5 is buffer
                    //strafe(1);
                    strafe(-1 * xPosNew);
                }
                if (tagProcessor2.getDetections().get(0).ftcPose.x > (0.5 + x)) { //0.5 is buffer
                    //strafe(-1);
                    strafe(-1 * xPosNew);
                }
            }
        }
    }

    public void alignY(double y, int vision) {
        double yPosNew;
        //double moveInRevs;
        //alignX(-1, 1, 12);
        if (vision == 1) {
            if (tagProcessor.getDetections().size() > 0) {
                yPosNew = tagProcessor.getDetections().get(0).ftcPose.y - y;
                //moveInRevs = yPosNew / CIRCUMFERENCE_INCHES;

                if (tagProcessor.getDetections().get(0).ftcPose.y < (-0.5 + y)) { //0.5 is buffer
                    //strafe(1);
                    moveStraightLine(1 * yPosNew);
                }
                if (tagProcessor.getDetections().get(0).ftcPose.y > (0.5 + y)) { //0.5 is buffer
                    //strafe(-1);
                    moveStraightLine(1 * yPosNew);
                }
            }
        } else if (vision == 2) {
            if (tagProcessor2.getDetections().size() > 0) {
                yPosNew = tagProcessor2.getDetections().get(0).ftcPose.y - y;
                //moveInRevs = yPosNew / CIRCUMFERENCE_INCHES;

                if (tagProcessor2.getDetections().get(0).ftcPose.y < (-0.5 + y)) { //0.5 is buffer
                    //strafe(1);
                    moveStraightLine(-1 * yPosNew);
                }
                if (tagProcessor2.getDetections().get(0).ftcPose.y > (0.5 + y)) { //0.5 is buffer
                    //strafe(-1);
                    moveStraightLine(-1 * yPosNew);
                }
            }
        }
    }

    private boolean setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure. Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

    }

    public void alignSample() {

        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(500, 500000, blobs);
        sleep(200);


        boolean centered = false;
        if (!blobs.isEmpty()) {

            org.opencv.core.Size myBoxFitSize;
            double boxWidth = 0.0;
            double boxHeight = 0.0;
            while (!centered) {
                RotatedRect boxFit = blobs.get(0).getBoxFit();
                myBoxFitSize = boxFit.size;
                boxWidth = myBoxFitSize.width;
                boxHeight = myBoxFitSize.height;
                int currX = (int) boxFit.center.x;
                double error = 320 - currX;
                if (Math.abs((currX) - 320) <= 20) {
                    centered = true;
                    strafe(5);
                } else if (Math.abs((currX) - 320) <= 100) {
                    strafe(-1 * error / 40);
                } else {
                    strafe(-1 * error / 20);
                }
                telemetry.addLine(String.valueOf((int) boxFit.center.x));
            }
            double distance = 18644 / Math.min(boxHeight, boxWidth);
            while (Math.abs(distance - 340) <= 10) {
                moveStraightLine(distance-340);
            }
        }





        sleep(20);
        telemetry.update();
    }

    public void moveLinearSlideRevs(double y) {
        double up;
        if (y > 0) {
            while (linearSlideMotor.getCurrentPosition() < 3064 && linearSlideMotor.getCurrentPosition() < y) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //hangSlideMotor.setPower(1* UtilityValues.LSSPEED);
                up = Math.sin(((double) (4000 - linearSlideMotor.getCurrentPosition()) / 4000) * Math.PI / 2);
                linearSlideMotor.setPower(/*UtilityValues.LSSPEED * */up*gamepad2.right_trigger);
            }
            while (linearSlideMotor.getCurrentPosition() > 3064) {
                linearSlideMotor.setPower(-0.3);
            }
            linearSlideMotor.setPower(0);
        } else if (y < 0) {
            while (linearSlideMotor.getCurrentPosition() > 0 && linearSlideMotor.getCurrentPosition() > y) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ///hangSlideMotor.setPower(-1*UtilityValues.LSSPEED);
                up = Math.sin(((double) (1000+linearSlideMotor.getCurrentPosition()) /4000)*Math.PI/2);
                linearSlideMotor.setPower(-1* /*UtilityValues.LSSPEED**/up*gamepad2.left_trigger);
            }
            while (linearSlideMotor.getCurrentPosition() < 0) {
                linearSlideMotor.setPower(-0.3);
            }
            linearSlideMotor.setPower(0);
        }
    }

    public void moveLinearSlide(double inches) {
        double inchesWithoutRobotHeight = inches - 3;
        if (inchesWithoutRobotHeight < 0) {
            inchesWithoutRobotHeight = 0;
        }
        //double mm = inches * 25.4;
        double y; // = mm * (984.0 / 3064.0);
        y = inchesWithoutRobotHeight * (3064.0 / 40.5);
        double up;
        if (y > 0) {
            while (linearSlideMotor.getCurrentPosition() < 3064 && linearSlideMotor.getCurrentPosition() < y) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //hangSlideMotor.setPower(1* UtilityValues.LSSPEED);
                up = Math.sin(((double) (4000 - linearSlideMotor.getCurrentPosition()) / 4000) * Math.PI / 2);
                linearSlideMotor.setPower(/*UtilityValues.LSSPEED * */up*gamepad2.right_trigger);
            }
            while (linearSlideMotor.getCurrentPosition() > 3064) {
                linearSlideMotor.setPower(-0.3);
            }
            linearSlideMotor.setPower(0);
        } else if (y < 0) {
            while (linearSlideMotor.getCurrentPosition() > 0 && linearSlideMotor.getCurrentPosition() > y) {
                linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ///hangSlideMotor.setPower(-1*UtilityValues.LSSPEED);
                up = Math.sin(((double) (1000+linearSlideMotor.getCurrentPosition()) /4000)*Math.PI/2);
                linearSlideMotor.setPower(-1* /*UtilityValues.LSSPEED**/up*gamepad2.left_trigger);
            }
            while (linearSlideMotor.getCurrentPosition() < 0) {
                linearSlideMotor.setPower(-0.3);
            }
            linearSlideMotor.setPower(0);
        }
    }

    public void rotate(double degrees) {

        double robotSpeed = SPEED;
        // Assume positive degrees means moving towards the right
        double movementOfWheelsInRevs = Math.abs(degrees / DEGREES_MOTOR_MOVES_IN_1_REV);

        if (degrees >= 0) {
            drive(robotSpeed,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs);
        } else {
            // Moving negative means rotating left
            drive(robotSpeed,
                    -1 * movementOfWheelsInRevs,
                    -1 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs,
                    1.0 * movementOfWheelsInRevs);
        }
    }

    private void strafe(double strafeInches) {
        // We assume that strafing right means positive
        double strafeRevs = Math.abs(strafeInches / CIRCUMFERENCE_INCHES);
        if (strafeInches >= 0) {
            telemetry.addData("Strafing towards right by ", "%.3f inches", strafeInches);

            drive(SPEED,
                    1 * strafeRevs,
                    -1 * strafeRevs,
                    -1 * strafeRevs,
                    1 * strafeRevs);
        } else {
            telemetry.addData("Strafing towards Left by ", "%.3f inches", Math.abs(strafeInches));

            drive(SPEED,
                    -1 * strafeRevs,
                    1 * strafeRevs,
                    1 * strafeRevs,
                    -1 * strafeRevs);
        }
    }

    /*
    =====================================================
    MOVE IN STRAIGHT LINE FUNCTION
    to call:
        moveStraightLine(# of inches);
        positive # of inches -> forward
    =====================================================
    */
    private void moveStraightLine(double movementInInches) {
        double moveInRevs = movementInInches / CIRCUMFERENCE_INCHES;
        telemetry.addData("Moving ", "%.3f inches", movementInInches);
        telemetry.update();
        drive(SPEED, moveInRevs, moveInRevs, moveInRevs, moveInRevs);
    }

    public void drive(double speed, double leftFrontRevs, double leftBackRevs, double rightFrontRevs, double rightBackRevs) {

        int LFdrivetarget = (int) (leftFrontRevs * MOTOR_TICK_COUNTS) + leftFrontDrive.getCurrentPosition();
        int LBdrivetarget = (int) (leftBackRevs * MOTOR_TICK_COUNTS) + leftBackDrive.getCurrentPosition();
        int RFdrivetarget = (int) (rightFrontRevs * MOTOR_TICK_COUNTS) + rightFrontDrive.getCurrentPosition();
        int RBdrivetarget = (int) (rightBackRevs * MOTOR_TICK_COUNTS) +  rightBackDrive.getCurrentPosition();

        leftFrontDrive.setTargetPosition(LFdrivetarget);
        leftBackDrive.setTargetPosition(LBdrivetarget);
        rightFrontDrive.setTargetPosition(RFdrivetarget);
        rightBackDrive.setTargetPosition(RBdrivetarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
//            telemetry.addLine("Current Position of the Motors")
//                    .addData("Left Front  ", "%d", leftFrontDrive.getCurrentPosition())
//                    .addData("Left Back ", "%d", leftBackDrive.getCurrentPosition())
//                    .addData("Right Front ", "%d", rightFrontDrive.getCurrentPosition())
//                    .addData("Right Back ", "%df", rightBackDrive.getCurrentPosition());
//
//            telemetry.addLine("Target Positions of the Motors")
//                    .addData("Left Front  ", "%d", LFdrivetarget)
//                    .addData("Left Back ", "%d", LBdrivetarget)
//                    .addData("Right Front ", "%d", RFdrivetarget)
//                    .addData("Right Back ", "%df", RBdrivetarget);

            //telemetry.update();
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);


        sleep(250);
    }

    public void strafeDiagonalLeft(double strafeLeftInches) {
        double robotSpeed = SPEED;

        double strafeLeftRevs = Math.abs(strafeLeftInches / CIRCUMFERENCE_INCHES);

        if (strafeLeftInches >= 0) {
            drive(robotSpeed,
                    0,
                    1 * strafeLeftRevs,
                    1 * strafeLeftRevs,
                    0);
        } else {
            drive(robotSpeed,
                    0,
                    -1 * strafeLeftRevs,
                    -1 * strafeLeftRevs,
                    0);
        }
    }

    public void strafeDiagonalRight(double strafeLeftInches) {

        double robotSpeed = SPEED;
        double strafeLeftRevs = Math.abs(strafeLeftInches / CIRCUMFERENCE_INCHES);

        if (strafeLeftInches >= 0) {
            drive(robotSpeed,
                    1 * strafeLeftRevs,
                    0,
                    0,
                    1 * strafeLeftRevs);
        } else {
            drive(robotSpeed,
                    -1 * strafeLeftRevs,
                    0,
                    0,
                    -1 * strafeLeftRevs);
        }
    }

}