
package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.hiveShoot;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v1.PathsV1;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.HiveTilt;
import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.TiltState;

@Autonomous(name = "BioBuzz Hive Shoot Auto Ivy", group = "BioBuzz")
public class HiveShootAutoIvy extends LinearOpMode {

    // =============================================
    // CONFIGURATION
    // =============================================

    private static final Hive MY_HIVE = Hive.RED;

    // Keep the original autonomous starting pose.
    private static final Pose START_POSE = PathsV1.START_POSE;

    // Audience cell up: bottom shooting position.
    private static final double SHOOT_BOTTOM_X = 55.4255751014885;
    private static final double SHOOT_BOTTOM_Y = 7.808525033829501;

    // Scoring cell up: top shooting position.
    private static final double SHOOT_TOP_X = 42.97970230040596;
    private static final double SHOOT_TOP_Y = 110.05615696887686;

    // Keeps the top path left of the HIVE frame.
    private static final Pose TOP_PATH_CONTROL =
            new Pose(26.0, 60.0);

    private static final int CONFIRM_FRAMES = 5;

    private static final double SCAN_AFTER_START_SEC = 0.5;
    private static final double PATH_TIMEOUT_SEC = 6.0;
    private static final double SHOOT_TIME_SEC = 3.0;

    // =============================================
    // HARDWARE
    // =============================================

    private Follower follower;
    private Limelight3A limelight;

    // =============================================
    // HIVE DETECTION STATE
    // =============================================

    private TiltState confirmedState = null;
    private TiltState candidateState = null;

    private int candidateFrames = 0;

    private HiveTilt lastTilt = null;

    // =============================================
    // IVY STATE
    // =============================================

    private String currentStatus = "Initializing";

    private boolean pathTimedOut = false;

    private final ElapsedTime shootTimer = new ElapsedTime();

    // =============================================
    // MAIN AUTONOMOUS
    // =============================================

    @Override
    public void runOpMode() {

        // Ivy uses a static scheduler.
        // Reset it before starting a new OpMode.
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
        follower.update();

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(
                HiveTagGeometry.APRILTAG_PIPELINE
        );
        limelight.start();

        try {

            // =====================================
            // INIT: SCAN HIVE
            // =====================================

            while (opModeInInit()) {

                scanHive();

                telemetry.addLine(
                        "BioBuzz Hive Shoot Auto Ivy - scanning "
                                + MY_HIVE + " HIVE"
                );

                addScanTelemetry();
                telemetry.update();
            }

            if (isStopRequested()) {
                return;
            }

            // =====================================
            // FINAL SCAN AFTER START
            // =====================================

            ElapsedTime scanTimer = new ElapsedTime();

            while (opModeIsActive()
                    && scanTimer.seconds() < SCAN_AFTER_START_SEC) {

                follower.update();
                scanHive();

                telemetry.addLine("Final HIVE scan");

                addScanTelemetry();
                telemetry.update();
            }

            limelight.stop();

            if (!opModeIsActive()) {
                return;
            }

            // =====================================
            // SELECT SHOOTING POSITION
            // =====================================

            TiltState decision =
                    confirmedState != null
                            ? confirmedState
                            : TiltState.AUDIENCE_UP;

            boolean shootTop =
                    decision == TiltState.SCORING_UP;

            Pose shootPose = shootTop
                    ? facingHive(SHOOT_TOP_X, SHOOT_TOP_Y)
                    : facingHive(SHOOT_BOTTOM_X, SHOOT_BOTTOM_Y);

            Path toShoot = shootTop
                    ? curve(
                    START_POSE,
                    TOP_PATH_CONTROL,
                    shootPose
            ).linear(START_POSE, shootPose)

                    : line(
                    START_POSE,
                    shootPose
            ).linear(START_POSE, shootPose);

            String spot = shootTop ? "TOP" : "BOTTOM";

            String source = confirmedState != null
                    ? "tags"
                    : "default (tags not read)";

            // =====================================
            // BUILD IVY COMMAND
            // =====================================

            pathTimedOut = false;

            Command autoCommand = autoRoutine(
                    toShoot,
                    spot
            );

            // =====================================
            // SCHEDULE AUTONOMOUS
            // =====================================

            currentStatus = "Starting autonomous";

            schedule(autoCommand);

            // =====================================
            // MAIN IVY LOOP
            // =====================================

            while (opModeIsActive()) {

                // Update Pedro localization and movement.
                follower.update();

                // Execute Ivy commands.
                Scheduler.execute();

                // Update telemetry.
                addStatusTelemetry(
                        currentStatus,
                        decision,
                        spot,
                        source
                );

                telemetry.addData(
                        "Path Timed Out",
                        pathTimedOut
                );

                telemetry.addData(
                        "Ivy Scheduled",
                        autoCommand.isScheduled()
                );

                telemetry.update();

                idle();
            }

        } finally {

            // Cancel any remaining Ivy commands.
            Scheduler.reset();

            // Stop drivetrain and vision hardware.
            follower.stop();
            limelight.stop();
        }
    }

    // =============================================
    // IVY AUTONOMOUS ROUTINE
    // =============================================

    private Command autoRoutine(Path path, String spot) {

        return sequential(

                // Stage 1: Drive to the selected position.
                instant(() -> {
                    currentStatus = "Driving to " + spot;
                }),

                // Race the path against the six-second timeout.
                race(

                        // Normal completion: Pedro finishes following.
                        follow(follower, path),

                        // Timeout: stop the follower and mark failure.
                        sequential(
                                waitMs(
                                        (long) (PATH_TIMEOUT_SEC * 1000)
                                ),

                                instant(() -> {

                                    pathTimedOut = true;

                                    currentStatus =
                                            "Path timed out - stopping robot";

                                    follower.stop();
                                })
                        )
                ),

                // Stage 2: Shooting placeholder.
                shootingPlaceholder(),

                // Stage 3: Finish.
                instant(() -> {

                    currentStatus = pathTimedOut
                            ? "Done - path timed out, shooting skipped"
                            : "Done";
                })
        );
    }

    // =============================================
    // SHOOTING PLACEHOLDER COMMAND
    // =============================================

    private Command shootingPlaceholder() {

        return Command.build()

                .setStart(() -> {

                    if (!pathTimedOut) {

                        currentStatus = "Shooting";

                        shootTimer.reset();
                    }
                })

                .setExecute(() -> {
                    // Physical shooter control will go here.
                })

                .setDone(() -> {

                    // If the path timed out, skip shooting.
                    if (pathTimedOut) {
                        return true;
                    }

                    // Otherwise wait for the original 3 seconds.
                    return shootTimer.seconds() >= SHOOT_TIME_SEC;
                })

                .setEnd(endCondition -> {
                    // Nothing to stop yet because the shooter
                    // is currently only a placeholder.
                });
    }

    // =============================================
    // HIVE GEOMETRY
    // =============================================

    /**
     * Aim the robot front toward the center of our HIVE.
     * Heading is calculated using atan2.
     */
    private Pose facingHive(double x, double y) {

        double hiveX = MY_HIVE == Hive.RED
                ? HiveTagGeometry.RED_HIVE_X_IN
                : HiveTagGeometry.BLUE_HIVE_X_IN;

        double heading = Math.atan2(
                HiveTagGeometry.PIVOT_Y_IN - y,
                hiveX - x
        );

        return new Pose(x, y, heading);
    }

    // =============================================
    // HIVE APRILTAG SCANNING
    // =============================================

    private void scanHive() {

        lastTilt = HiveTagGeometry.measureHive(
                HiveTagGeometry.readTags(
                        limelight.getLatestResult()
                ),
                MY_HIVE
        );

        TiltState state = lastTilt.state;

        if (state != TiltState.SCORING_UP
                && state != TiltState.AUDIENCE_UP) {
            return;
        }

        if (state == candidateState) {

            candidateFrames++;

        } else {

            candidateState = state;
            candidateFrames = 1;
        }

        if (candidateFrames >= CONFIRM_FRAMES) {
            confirmedState = state;
        }
    }

    // =============================================
    // SCAN TELEMETRY
    // =============================================

    private void addScanTelemetry() {

        telemetry.addData(
                "Confirmed",
                confirmedState == null
                        ? "none yet"
                        : confirmedState.label
        );

        if (lastTilt != null && lastTilt.tagCount > 0) {

            telemetry.addData(
                    "Live",
                    "%s (%.1f degrees, %d tags)",
                    lastTilt.state.label,
                    lastTilt.tiltDeg,
                    lastTilt.tagCount
            );

        } else {

            telemetry.addData(
                    "Live",
                    "no " + MY_HIVE + " tags visible"
            );
        }

        boolean top =
                confirmedState == TiltState.SCORING_UP;

        telemetry.addData(
                "Will shoot from",
                top
                        ? "TOP (%.1f, %.1f)"
                        : "BOTTOM (%.1f, %.1f)",
                top ? SHOOT_TOP_X : SHOOT_BOTTOM_X,
                top ? SHOOT_TOP_Y : SHOOT_BOTTOM_Y
        );
    }

    // =============================================
    // AUTONOMOUS STATUS TELEMETRY
    // =============================================

    private void addStatusTelemetry(
            String status,
            TiltState decision,
            String spot,
            String source
    ) {

        Pose pose = follower.pose();

        telemetry.addData("Status", status);

        telemetry.addData(
                "HIVE",
                "%s (from %s)",
                decision.label,
                source
        );

        telemetry.addData("Shoot spot", spot);

        telemetry.addData(
                "Pose",
                "(%.1f, %.1f) %.1f degrees",
                pose.x(),
                pose.y(),
                Math.toDegrees(pose.heading())
        );
    }
}
