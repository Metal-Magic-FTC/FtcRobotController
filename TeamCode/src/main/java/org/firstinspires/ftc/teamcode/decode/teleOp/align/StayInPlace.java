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


                drivetrain.driveMecanum(strafe, drive, turn);

                if (drive+strafe+turn==0) {
                    if (oneTime) {
                        savePose = follower.getPose();
                        oneTime = false;
                    } else {
                        follower.setPose(savePose);
                    }
                } else {
                    oneTime = true;
                }

            }
        }

}
