package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.ml.ParamGrid;

@TeleOp(name = "!Sensor: LimeLightDistance", group = "Sensor")
@Disabled
public class limeLightDistance extends OpMode {
    private Limelight3A limelight;
    //TestBench bench = new TestBench();
   // private IMU imu;
    private double distance;
    GoBildaPinpointDriver odo;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        //imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot
                (RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        //imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }
    public void start() {
        limelight.start();
    }
    @Override
    public void loop() {
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Orientation", odo.getHeading(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();
        telemetry.addData("Pipeline: ", llResult.getPipelineIndex());
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();
            distance = 9315/llResult.getTa();
            telemetry.addData("Calculated Distance: ", distance);
            telemetry.addData("Target X: ", llResult.getTx());
            telemetry.addData("Target Area: ", llResult.getTa());
            telemetry.addData("Botpose: ", botpose.toString());
        }
    }
}
