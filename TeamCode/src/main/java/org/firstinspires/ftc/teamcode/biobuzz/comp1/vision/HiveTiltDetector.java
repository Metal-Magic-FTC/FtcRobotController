package org.firstinspires.ftc.teamcode.biobuzz.comp1.vision;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.HiveTilt;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TagObservation;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TiltState;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedroPathing.Constants;

import java.util.List;

/**
 * Program 1: looks at the AprilTags under the HIVE cells and reports which way each HIVE is tipped
 * (scoring cell up, audience cell up, or in transition) plus the tilt angle from the ground.
 *
 * Drive with gamepad1 (robot-centric) to point the Limelight at a HIVE.
 * See HiveTagGeometry for the math and the camera constants you need to measure.
 */
@TeleOp(name = "HIVE Tilt Detector", group = "BioBuzz")
public class HiveTiltDetector extends LinearOpMode {

    /** Exponential smoothing on the tilt reading (0..1, higher = faster/noisier). */
    private static final double SMOOTHING = 0.35;
    /** Forget a HIVE's reading if it has not been seen for this long. */
    private static final double FORGET_AFTER_SEC = 0.75;

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(HiveTagGeometry.APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("HIVE Tilt Detector ready");
        telemetry.addData("Camera height / pitch", "%.1f in / %.1f°",
                HiveTagGeometry.CAMERA_HEIGHT_IN, HiveTagGeometry.CAMERA_PITCH_DEG);
        telemetry.update();
        waitForStart();

        follower.startTeleopDrive();
        ElapsedTime clock = new ElapsedTime();
        double[] filteredTilt = new double[Hive.values().length];
        double[] lastSeen = {-100, -100};

        while (opModeIsActive()) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            double now = clock.seconds();
            List<TagObservation> tags = HiveTagGeometry.readTags(limelight.getLatestResult());

            for (Hive hive : Hive.values()) {
                int i = hive.ordinal();
                HiveTilt raw = HiveTagGeometry.measureHive(tags, hive);
                if (raw.tagCount > 0) {
                    boolean staleFilter = now - lastSeen[i] > FORGET_AFTER_SEC;
                    filteredTilt[i] = staleFilter
                            ? raw.tiltDeg
                            : filteredTilt[i] + SMOOTHING * (raw.tiltDeg - filteredTilt[i]);
                    lastSeen[i] = now;
                }

                boolean visible = now - lastSeen[i] <= FORGET_AFTER_SEC;
                TiltState state = visible ? HiveTagGeometry.classify(filteredTilt[i]) : TiltState.UNKNOWN;

                telemetry.addLine("=== " + hive + " HIVE ===");
                telemetry.addData("State", state.label);
                if (visible) {
                    telemetry.addData("Tilt from ground", "%.1f°  (+ = scoring cell up)", filteredTilt[i]);
                    telemetry.addData("Tags this frame", raw.tagCount);
                }
            }

            telemetry.addLine();
            telemetry.addLine("=== TAGS ===");
            if (tags.isEmpty()) telemetry.addLine("No HIVE tags (IDs 30-45) visible");
            for (TagObservation t : tags) {
                telemetry.addData("ID " + t.info.id, "%s %s | height %.1f in | range %.0f in | tilt %.1f°",
                        t.info.hive, t.info.cell, t.height, t.rangeIn, t.tiltDeg);
            }
            telemetry.update();
        }

        limelight.stop();
    }
}
