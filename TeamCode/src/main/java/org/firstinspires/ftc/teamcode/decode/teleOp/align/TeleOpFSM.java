package org.firstinspires.ftc.teamcode.decode.teleOp.align;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.decode.teleOp.actual.TeleV3;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
@TeleOp(name = "Drive + Hold + Face Goal", group = "Drive")
public class TeleOpFSM extends LinearOpMode {

    enum HoldState {
        MANUAL,
        HOLD,
        FACE
    }

    HoldState holdState = HoldState.MANUAL;

    Pose savePose;
    boolean holdInitialized = false;

    // Assume these already exist in your project
    private CustomMecanumDrive drivetrain;
    private Follower follower;

    static final Pose GOAL_POSE = new Pose(72, 36, 0); // example goal location

    @Override
    public void runOpMode() {
        drivetrain = new CustomMecanumDrive(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            /* -------- STATE SWITCHING -------- */
            if (gamepad1.a) {
                holdState = HoldState.MANUAL;
                holdInitialized = false;
            }

            if (gamepad1.b) {
                holdState = HoldState.HOLD;
                holdInitialized = false;
            }

            if (gamepad1.y) {
                holdState = HoldState.FACE;
                holdInitialized = false;
            }

            /* -------- STATE LOGIC -------- */
            switch (holdState) {

                case MANUAL:
                    double drive = -gamepad1.left_stick_y;
                    double strafe = gamepad1.left_stick_x;
                    double turn = gamepad1.right_stick_x;

                    drivetrain.driveMecanum(strafe, drive, turn);
                    break;

                case HOLD:
                    follower.update();

                    if (!holdInitialized) {
                        savePose = follower.getPose();
                        follower.holdPoint(savePose);
                        holdInitialized = true;
                    }
                    break;

                case FACE:
                    follower.update();

                    if (!holdInitialized) {
                        savePose = follower.getPose();
                        savePose = savePose.setHeading(faceGoal(savePose, GOAL_POSE));
                        follower.holdPoint(savePose);
                        holdInitialized = true;
                    }
                    break;
            }

            telemetry.addData("State", holdState);
            telemetry.update();
        }
    }

    /* -------- Helper Function -------- */
    private double faceGoal(Pose robotPose, Pose goalPose) {
        return Math.atan2(
                goalPose.getY() - robotPose.getY(),
                goalPose.getX() - robotPose.getX()
        );
    }
}
