package org.firstinspires.ftc.teamcode.decode.pedroPathing.statesAutos.red;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "!!!!!! !!!!!!! TwelveBallRedClose")
@Configurable // Panels
public class TwelveBallRedClose extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private boolean startedPath = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(118.157, 128.629, Math.toRadians(45)));

        paths = new Paths(follower);

        pathState = 0;
        startedPath = false;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain scan;
        public PathChain shoot;
        public PathChain tointake1;
        public PathChain intake1;
        public PathChain gate;
        public PathChain shoot2;
        public PathChain tointake2;
        public PathChain intake2;
        public PathChain shoot3;
        public PathChain tointake3;
        public PathChain intake3;
        public PathChain shoot4;
        public PathChain leave;

        public Paths(Follower follower) {
            scan = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118.157, 128.629),

                                    new Pose(105.438, 116.360)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(125))

                    .build();

            shoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(105.438, 116.360),

                                    new Pose(91.371, 102.034)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(45))

                    .build();

            tointake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(91.371, 102.034),

                                    new Pose(99.899, 83.809)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.899, 83.809),

                                    new Pose(119.787, 79)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            gate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.787, 79),

                                    new Pose(127.5, 78)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.5, 79),

                                    new Pose(91.371, 102.034)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))

                    .build();

            tointake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(91.371, 102.034),
                                    new Pose(68.8707865168539, 60.123595505617985),
                                    new Pose(103.31460674157304, 55)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.899, 55),

                                    new Pose(119.787, 55)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.787, 55),

                                    new Pose(91.371, 102.034)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            tointake3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(91.371, 102.034),
                                    new Pose(68.58988764044946, 35.7),
                                    new Pose(103.85393258426967, 30)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            intake3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.899, 30),

                                    new Pose(119.787, 30)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shoot4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.787, 30),

                                    new Pose(91.371, 102.034)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(91.371, 102.034),

                                    new Pose(96.831, 70.730)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();
        }
    }

//    @Override
//    public void start() {
//        pathState = 0;
//        startedPath = false;
//    }

    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0: // Scan
                if (!startedPath) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.scan);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 1: // Shoot
                if (!startedPath) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.shoot);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 2: // To Intake 1
                if (!startedPath) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.tointake1);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 3: // Intake 1
                if (!startedPath) {
                    follower.setMaxPower(0.35);
                    follower.followPath(paths.intake1);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 4: // Gate
                if (!startedPath) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.gate);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 5: // Shoot 2
                if (!startedPath) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.shoot2);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 6: // To Intake 2
                if (!startedPath) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.tointake2);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 7: // Intake 2
                if (!startedPath) {
                    follower.setMaxPower(0.35);
                    follower.followPath(paths.intake2);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 8: // Shoot 3
                if (!startedPath) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.shoot3);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 9: // To Intake 3
                if (!startedPath) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.tointake3);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 10: // Intake 3
                if (!startedPath) {
                    follower.setMaxPower(0.35);
                    follower.followPath(paths.intake3);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 11: // Shoot 4
                if (!startedPath) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.shoot4);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            case 12: // Leave
                if (!startedPath) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.leave);
                    startedPath = true;
                }
                if (!follower.isBusy()) {
                    pathState++;
                    startedPath = false;
                }
                break;

            default:
                // Auto complete — do nothing
                break;
        }

        return pathState;
    }
}
