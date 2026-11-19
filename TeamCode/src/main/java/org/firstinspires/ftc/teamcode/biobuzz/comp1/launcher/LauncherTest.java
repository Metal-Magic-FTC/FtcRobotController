//package org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.LaunchPlanner.Plan;
//import org.firstinspires.ftc.teamcode.biobuzz.comp1.pedro.Constants;
//import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Cell;
//import org.firstinspires.ftc.teamcode.biobuzz.comp1.vision.HiveTagGeometry.Hive;
//
///**
// * RED launcher test: one flywheel motor, hood set BY HAND, launcher fixed to the chassis (aim by turning the robot).
// * Pedro tracks the robot from a known start pose; you enter the hood angle you set, and the flywheel speed follows
// * the distance to the RED HIVE's upward-facing CELL.
// *
// * BEFORE PRESSING INIT: robot in the bottom-left corner (left side on the red wall, back on the audience wall),
// * facing AWAY from the audience, and still while the Pinpoint calibrates.
// *
// * gamepad1:
// *   left stick / right stick X - drive (robot-centric) / turn, right bumper - slow
// *   Y           - flywheel on / off
// *   dpad up / down    - hood angle you set +/- 1 deg (hold left bumper for 0.25 deg)
// *   B           - copy the SUGGESTED hood angle into the input (then set the real hood to match)
// *   A           - which red CELL is up: SCORING (far side) <-> AUDIENCE
// *   dpad right / left - speed trim +/- 1% (tuning: when shots score, copy "scale" into SPEED_SCALE for that distance)
// *   back        - reset speed trim to 1.00
// *   X           - reset pose to the start corner (put the robot there first)
// */
//@TeleOp(name = "BioBuzz Launcher Test (Red)", group = "BioBuzz Tests")
//public class LauncherTest extends LinearOpMode {
//
//    private static final Pose START_POSE = new Pose(Constants.ROBOT_WIDTH_IN / 2.0, Constants.ROBOT_LENGTH_IN / 2.0,
//            Math.toRadians(90));
//    private static final double SLOW_MODE_SCALE = 0.4;
//    private static final double STABLE_TILT_DEG = 30.0;
//
//    @Override
//    public void runOpMode() {
//        Launcher launcher = new Launcher(hardwareMap);
//
//        telemetry.addLine("Calibrating Pinpoint - keep the robot STILL");
//        telemetry.update();
//        Follower follower = Constants.create(hardwareMap);
//        follower.setPose(START_POSE);
//
//        double tiltDeg = STABLE_TILT_DEG; // scoring cell up
//        double hoodDeg = LauncherConstants.HOOD_DEFAULT_DEG;
//        double trim = 1.0;
//        boolean spinning = false;
//        double lastGoodMps = 0;
//
//        while (opModeInInit()) {
//            follower.update();
//            Pose pose = follower.pose();
//            telemetry.addLine("Robot in the bottom-left corner facing away from the audience?");
//            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
//            telemetry.update();
//        }
//
//        while (opModeIsActive()) {
//            double s = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;
//            follower.manual(-gamepad1.left_stick_y * s, -gamepad1.left_stick_x * s, -gamepad1.right_stick_x * s);
//            follower.update();
//            Pose pose = follower.pose();
//
//            double hoodStep = gamepad1.left_bumper ? 0.25 : 1.0;
//            if (gamepad1.dpadUpWasPressed()) hoodDeg += hoodStep;
//            if (gamepad1.dpadDownWasPressed()) hoodDeg -= hoodStep;
//            hoodDeg = Range.clip(hoodDeg, LauncherConstants.HOOD_MIN_DEG, LauncherConstants.HOOD_MAX_DEG);
//            if (gamepad1.yWasPressed()) spinning = !spinning;
//            if (gamepad1.aWasPressed()) tiltDeg = -tiltDeg;
//            if (gamepad1.dpadRightWasPressed()) trim += 0.01;
//            if (gamepad1.dpadLeftWasPressed()) trim -= 0.01;
//            if (gamepad1.backWasPressed()) trim = 1.0;
//            if (gamepad1.xWasPressed()) follower.setPose(START_POSE);
//
//            Plan plan = LaunchPlanner.plan(pose.x(), pose.y(), pose.heading(), Hive.RED, tiltDeg, hoodDeg, trim);
//            if (gamepad1.bWasPressed() && !Double.isNaN(plan.suggestedHoodDeg)) {
//                hoodDeg = Math.round(plan.suggestedHoodDeg * 4.0) / 4.0; // nearest 0.25 deg
//                plan = LaunchPlanner.plan(pose.x(), pose.y(), pose.heading(), Hive.RED, tiltDeg, hoodDeg, trim);
//            }
//
//            // Impossible shots (too close, behind the cell, ...) keep the last good speed instead of chasing junk
//            if (plan.possible) lastGoodMps = plan.commandedBallMps;
//            if (spinning && lastGoodMps > 0) launcher.setBallSpeed(lastGoodMps);
//            else launcher.stop();
//
//            boolean aimed = Math.abs(plan.headingErrorDeg) <= LauncherConstants.AIM_TOLERANCE_DEG;
//            boolean ready = spinning && plan.possible && aimed && !launcher.saturated() && launcher.ready();
//
//            telemetry.addData("READY", ready ? ">>> FIRE <<<" : "no");
//            telemetry.addData("Problem", plan.possible ? (launcher.saturated() ? "flywheel at max speed" : "-") : plan.problem);
//            telemetry.addData("Hood SET (dpad U/D)", "%.2f°", hoodDeg);
//            telemetry.addData("Hood suggested (B)", Double.isNaN(plan.suggestedHoodDeg) ? "none in range"
//                    : String.format("%.1f°", plan.suggestedHoodDeg));
//            telemetry.addData("Target (A)", "%s cell up  (%.1f, %.1f, z %.1f)",
//                    plan.cell == Cell.SCORING ? "SCORING" : "AUDIENCE", plan.target[0], plan.target[1], plan.target[2]);
//            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
//            telemetry.addData("Aim", "turn %+.1f°  %s", plan.headingErrorDeg, aimed ? "(on target)" : "");
//            telemetry.addData("Distance / rise", "%.1f in / %.1f in   approach %.0f°",
//                    plan.distanceIn, plan.riseIn, plan.approachYawDeg);
//            if (plan.shot.valid) {
//                telemetry.addData("Shot", "ideal %.2f m/s  entry %.0f°  apex +%.0f in  %.2f s",
//                        plan.shot.ballSpeedMps, plan.shot.entryDescentDeg, plan.shot.apexAboveExitIn,
//                        plan.shot.flightTimeSec);
//            }
//            telemetry.addData("Speed scale (dpad L/R)", "%.3f  (table x trim %.2f)", plan.speedScale, trim);
//            telemetry.addData("Ball speed", "cmd %.2f  measured %.2f m/s", launcher.targetBallMps(),
//                    launcher.measuredBallMps());
//            telemetry.addData("Flywheel", "%.0f / %.0f rpm  %s", Launcher.radToRpm(launcher.motorRad()),
//                    Launcher.radToRpm(launcher.targetMotorRad()), launcher.ready() ? "(at speed)" : "");
//            telemetry.update();
//        }
//
//        launcher.stop();
//    }
//}
