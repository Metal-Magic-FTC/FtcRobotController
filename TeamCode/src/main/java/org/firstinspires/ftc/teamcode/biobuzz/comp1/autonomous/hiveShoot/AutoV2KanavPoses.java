package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.hiveShoot;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;

@Autonomous(name = "Auto V2 Follow Poses", group = "BioBuzz")
public class AutoV2KanavPoses extends LinearOpMode {

    private static final double PATH_TIMEOUT_SEC = 8.0;

    private static final PoseFactory P = PoseFactory.degrees();

    // Exact poses from your provided Paths class
    private final Pose start = P.of(31.5955, 137.2459, 270);
    private final Pose path1 = P.of(32.2457, 10.871, 270);

    private final Pose point2Start = P.of(32.2457, 10.871, 0);
    private final Pose point2 = P.of(56.5081, 19.9018, 90);

    private final Pose point3 = P.of(8.8735, 7.2687, -165.1466);
    private final Pose point4 = P.of(91.0029, 3.3399, -2.7387);

    private final Pose point5Start = P.of(91.0029, 3.3399, 0);
    private final Pose point5 = P.of(57.6825, 22.5708, 90);

    private final Pose point6 = P.of(4.539, 45.5017, 156.6603);

    private final Pose point7Start = P.of(4.539, 45.5017, 0);
    private final Pose point7 = P.of(57.5739, 28.0495, 90);

    private final Pose point8Start = P.of(57.5739, 28.0495, 0);
    private final Pose point8 = P.of(32.0211, 45.634, 90);

    private final Pose point9 = P.of(37.7871, 121.987, 85.6814);
    private final Pose point10 = P.of(46.9394, 136.536, 57.8272);
    private final Pose point11 = P.of(15.993, 32.5746, -106.5768);

    private final Pose point12Start = P.of(15.993, 32.5746, 0);
    private final Pose point12 = P.of(56.9214, 30.1859, 90);

    private Follower follower;

    @Override
    public void runOpMode() {

        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        // Build paths directly from the poses.
        Path[] sequence = {
                Paths.line(start, path1).linear(start, path1),
                Paths.line(point2Start, point2).linear(point2Start, point2),
                Paths.line(point2, point3).tangent(),
                Paths.line(point3, point4).tangent(),
                Paths.line(point5Start, point5).linear(point5Start, point5),
                Paths.line(point5, point6).tangent(),
                Paths.line(point7Start, point7).linear(point7Start, point7),
                Paths.line(point8Start, point8).linear(point8Start, point8),
                Paths.line(point8, point9).tangent(),
                Paths.line(point9, point10).tangent(),
                Paths.line(point10, point11).tangent(),
                Paths.line(point12Start, point12).linear(point12Start, point12)
        };

        while (opModeInInit()) {
            follower.update();

            telemetry.addLine("Auto V2 Follow Poses - READY");
            telemetry.addData("Start pose", "(%.2f, %.2f, %.1f°)",
                    start.x(), start.y(), Math.toDegrees(start.heading()));
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        for (int i = 0; i < sequence.length && opModeIsActive(); i++) {
            runPath(sequence[i], "Following path " + (i + 1));
        }

        while (opModeIsActive()) {
            follower.update();
            telemetry.addLine("Finished all paths");
            telemetry.addData("Final pose", follower.pose());
            telemetry.update();
        }
    }

    private void runPath(Path path, String status) {
        follower.follow(path);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && follower.isBusy()
                && timer.seconds() < PATH_TIMEOUT_SEC) {

            follower.update();

            telemetry.addData("Status", status);
            telemetry.addData("Pose", follower.pose());
            telemetry.update();
        }
    }
}