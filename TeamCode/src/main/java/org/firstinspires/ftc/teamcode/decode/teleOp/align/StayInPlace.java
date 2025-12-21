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

@TeleOp
public class StayInPlace extends LinearOpMode {

        private CustomMecanumDrive drivetrain;
        private Follower follower;
        private boolean oneTime;
        private Pose savePose;

        // ============================================================
        //                              INIT
        // ============================================================

        @Override
        public void runOpMode() throws InterruptedException {

            drivetrain = new CustomMecanumDrive(hardwareMap);

            waitForStart();
            follower = Constants.createFollower(hardwareMap);
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


                telemetry.addData("Move: ", drive+strafe+turn);
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
