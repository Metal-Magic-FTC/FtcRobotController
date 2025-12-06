package org.firstinspires.ftc.teamcode.limeLight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.json.JSONArray;
import org.json.JSONObject;
import java.net.URL;
import java.util.Scanner;

@TeleOp(name="Limelight Basic Pose", group="Vision")
@Disabled
public class LimelightPoseBasic extends LinearOpMode {

    // Change to your Limelight’s IP if needed
    private static final String LIMELIGHT_URL = "http://limelight.local:5807/api/v1/targets";

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            try {
                // Get JSON from Limelight
                String json = new Scanner(new URL(LIMELIGHT_URL).openStream(), "UTF-8").useDelimiter("\\A").next();
                JSONObject root = new JSONObject(json);

                JSONArray fiducials = root.getJSONObject("Results").getJSONArray("Fiducial");

                if (fiducials.length() > 0) {
                    JSONObject tag = fiducials.getJSONObject(0);
                    JSONArray pose = tag.getJSONArray("targetpose_cameraspace");

                    double x = pose.getDouble(0);
                    double y = pose.getDouble(1);
                    double z = pose.getDouble(2);
                    double roll  = pose.getDouble(3);
                    double pitch = pose.getDouble(4);
                    double yaw   = pose.getDouble(5);

                    telemetry.addLine("AprilTag Detected");
                    telemetry.addData("ID", tag.getInt("id"));
                    telemetry.addData("X (m)", x);
                    telemetry.addData("Y (m)", y);
                    telemetry.addData("Z (m)", z);
                    telemetry.addData("Roll (deg)", roll);
                    telemetry.addData("Pitch (deg)", pitch);
                    telemetry.addData("Yaw (deg)", yaw);
                } else {
                    telemetry.addLine("No AprilTag detected");
                }

            } catch (Exception e) {
                telemetry.addLine("Error reading Limelight");
                telemetry.addData("Exception", e.getMessage());
            }

            telemetry.update();
            sleep(100);
        }
    }
}
