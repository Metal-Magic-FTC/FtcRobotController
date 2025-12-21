package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.lang.Math;

public class CustomLimelight {
    private Limelight3A limelight;

    private double xPos; // 0 x is center
    private double yPos; // 0 y is center
    private double headingPos; // heading ranges from [0, 2pi]

    public CustomLimelight(HardwareMap hardwareMap) {
        // Declares limelight for vision tasks
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(3);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public Pose getLimelightPose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double heading = botpose.getOrientation().getYaw();

                xPos = x;
                yPos = y;
                headingPos = heading;

                return new Pose(x, y, heading);
            }

        }

        return null;
    }

    @Override
    public String toString() {
        return "Pose{x=" + xPos + ", y=" + yPos + ", heading=" + headingPos + "}";
    }

}
