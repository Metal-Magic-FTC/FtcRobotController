package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.*;
import org.firstinspires.ftc.teamcode.mmintothedeep.odometry.pinpoint.GoBildaPinpointDriver;

import java.util.List;


@TeleOp(name="April Tag Limelight Test")
public class AprilTagLimelightTest extends OpMode {

    private Limelight3A limelight3A;
//    GoBildaPinpointDriver odo;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(3); // april tag pipelining

//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//        odo.setOffsets(0, 130); //these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//
//        odo.resetPosAndIMU();

    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();

//        limelight3A.updateRobotOrientation(odo.getYawScalar());

        if (llResult != null && llResult.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                Pose3D targetCamera = fiducial.getTargetPoseCameraSpace(); // Not useful
                Pose3D robotField = fiducial.getRobotPoseFieldSpace(); // Useful
                Pose3D cameraTarget = fiducial.getCameraPoseTargetSpace(); // Useful
                Pose3D robotTarget = fiducial.getRobotPoseTargetSpace(); // Most useful
                Pose3D targetRobot = fiducial.getTargetPoseRobotSpace(); // Not useful

                telemetry.addData("id", id);
                telemetry.addData("targetCamera", targetCamera.toString());
                telemetry.addData("robotField", robotField.toString());
                telemetry.addData("cameraTarget", cameraTarget.toString());
                telemetry.addData("robotTarget", robotTarget.toString());
                telemetry.addData("targetRobot", targetRobot.toString());
            }

            Pose3D botpose = llResult.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            }


//            Pose3D botPose = llResult.getBotpose_MT2();
//
//            telemetry.addData("target X", llResult.getTx());
//            telemetry.addData("target Y", llResult.getTy());
//            telemetry.addData("target area", llResult.getTa());
//            telemetry.addData("botpose", botPose.toString());
//            telemetry.addData("orientation", botPose.getOrientation().toString()); // .getYaw etc
        }
    }
}
