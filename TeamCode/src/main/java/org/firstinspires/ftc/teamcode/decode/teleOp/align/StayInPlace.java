package org.firstinspires.ftc.teamcode.decode.teleOp.align;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.decode.teleOp.CustomMecanumDrive;

//
//@Disabled
@TeleOp
public class StayInPlace extends LinearOpMode {

        private CustomMecanumDrive drivetrain;
        private Follower follower;
        private boolean oneTime;


        public enum HoldState {
            MANUAL,
            HOLD,
            FACE
        };

        HoldState holdState = HoldState.MANUAL;

        private boolean faceGoal;
        private Pose savePose;
        private Pose START_POSE = new Pose (
                83,
                135,
                Math.toRadians(270)
        );

        private Pose GOAL_POSE = new Pose (
                136,
                136
        );

        // ============================================================
        //                              INIT
        // ============================================================

        @Override
        public void runOpMode() throws InterruptedException {

            drivetrain = new CustomMecanumDrive(hardwareMap);
            follower = Constants.createFollower(hardwareMap);

            waitForStart();
            oneTime = true;
            while (opModeIsActive()) {

                // ============================================================
                //                 GAMEPAD INPUT COLLECTION
                // ============================================================

                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                boolean idle =
                        Math.abs(drive) < 0.05 &&
                                Math.abs(strafe) < 0.05 &&
                                Math.abs(turn) < 0.05 && !(gamepad1.right_bumper);


                telemetry.addData("Move: ", drive + strafe + turn);
                if (idle) {

                    // HOLD POSITION
                    if (oneTime) {
                        follower.update();
                        savePose = follower.getPose();
                        oneTime = false;
                        follower.holdPoint(savePose);
                    }
                    telemetry.addLine("Holding Position");


                    follower.update();

                } else {
                    drivetrain.driveMecanum(strafe, drive, turn);
                    oneTime = true;
                    telemetry.addLine("Manual Drive");

                }

                telemetry.addData("Pose", follower.getPose());
                telemetry.update();

            }
        }


}
