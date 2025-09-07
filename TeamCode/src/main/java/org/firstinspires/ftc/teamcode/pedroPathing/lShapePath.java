//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;
//
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name="lShapePath")
//public class lShapePath extends OpMode {
//
//    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
//    private final Pose interPose = new Pose(24, 0, Math.toRadians(90));
//    private final Pose endPose = new Pose(24, 12, Math.toRadians(45));
//
//    private PathChain lShape;
//
//    /**
//     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//     * the Telemetry, as well as the Panels.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//        drawCurrentAndHistory();
//
//        if (follower.atParametricEnd()) {
//            follower.followPath(lShape, true);
//        }
//    }
//
//    @Override
//    public void init() {}
//
//    @Override
//    public void init_loop() {
//        telemetryM.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.");
//        telemetryM.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.");
//        telemetryM.update(telemetry);
//        follower.update();
//        drawCurrent();
//    }
//
//    /** Creates the PathChain for the "triangle".*/
//    @Override
//    public void start() {
//        follower.setStartingPose(startPose);
//
//        triangle = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, interPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
//                .addPath(new BezierLine(interPose, endPose))
//                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
//                .addPath(new BezierLine(endPose, startPose))
//                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
//                .build();
//
//        follower.followPath(triangle);
//    }
//}