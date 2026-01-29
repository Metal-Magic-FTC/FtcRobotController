package org.firstinspires.ftc.teamcode.decode.pedroPathing.optStatesAutos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@Autonomous(name = "!!! Close12BlueAuto")
@Configurable // Panels
public class Close12BlueAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private boolean startedPath = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(30.25, 132.75, Math.toRadians(90)));

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

        // Start position
        public Pose startPose = new Pose(30.25, 132.75, Math.toRadians(90));
        // Initial scoring position
        public Pose scanPose = new Pose(41, 105, Math.toRadians(50)); // score
        public Pose scorePose = new Pose(48, 96, Math.toRadians(135)); // score
        // Intaking first row
        public Pose tointake1Pose = new Pose(40,84, Math.toRadians(180)); // to intake
        public Pose tointake1ControlPose = new Pose(51,84, Math.toRadians(180)); // to intake
        public Pose intake1Pose = new Pose(24,84, Math.toRadians(180)); // intake
//        public Pose score2ControlPose = new Pose(56, 66);
        // Opening the gate
        public Pose gatePose = new Pose(17, 72.500, Math.toRadians(90)); //new Pose(144-132.781509, 61, Math.toRadians(28+90)); // gate
        public Pose gateControlPose = new Pose(55, 73); //62);
        // Intaking second row
        public Pose tointake2Pose = new Pose(40,60, Math.toRadians(180)); // to intake
        public Pose tointake2ControlPose = new Pose(55,56, Math.toRadians(180)); // to intake
        public Pose intake2Pose = new Pose(24, 60, Math.toRadians(180)); // intake
//        public Pose score3ControlPose = new Pose(56, 66);
        // Intaking the third row
        public Pose tointake3Pose = new Pose(40,35, Math.toRadians(180)); // to intake
        public Pose tointake3ControlPose = new Pose(55,31, Math.toRadians(180)); // to intake
        public Pose intake3Pose = new Pose(24, 35, Math.toRadians(180));
        public Pose score4ControlPose = new Pose(43, 78);

        // idk
        public Pose intakeCornerPose = new Pose(6.5,10 , Math.toRadians(270));
        public Pose intakeCornerControlPose = intakeCornerPose.withY(50);
        public Pose scoreToCornerPose = scorePose.withHeading(Math.toRadians(-135));
        public Pose scoreCornerPose = scorePose; //new Pose(56, 20, Math.toRadians(180));

        // leave
        public Pose leavePose = new Pose(48, 72, Math.toRadians(90));//new Pose(36, 12, Math.toRadians(180));

        public Paths(Follower follower) {
            // Start -> Scan
            scan = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    startPose,
                                    scanPose
                            )
                    )
                    .setNoDeceleration()
                    .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                    .build();

            // Scan -> Score (control)
            shoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scanPose,
                                    scorePose
                            )
                    )
                    .setNoDeceleration()
                    .setLinearHeadingInterpolation(scanPose.getHeading(), scorePose.getHeading())
                    .build();

            // Score -> toIntake (control)
            tointake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePose,
                                    tointake1ControlPose,
                                    tointake1Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose.getHeading(), tointake1Pose.getHeading(), 0.3)
                    .build();

            // toIntake -> intake
            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    tointake1Pose,
                                    intake1Pose
                            )
                    )
                    .setBrakingStrength(5)
                    .setBrakingStart(20)
                    .setLinearHeadingInterpolation(tointake1Pose.getHeading(), intake1Pose.getHeading(), 0.3)
                    .build();

            // intake -> gate (control)
            gate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    intake1Pose,
                                    gateControlPose,
                                    gatePose
                            )
                    )
                    .setBrakingStrength(3)
                    .setLinearHeadingInterpolation(intake1Pose.getHeading(), gatePose.getHeading(), 0.3)
                    .build();

            // gate -> score (score2 control)
            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    gatePose,
                                    scorePose
                            )
                    )
                    .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading(), 0.3)
                    .build();

            // score -> toIntake (control)
            tointake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePose,
                                    tointake2ControlPose,
                                    tointake2Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose.getHeading(), tointake2Pose.getHeading(), 0.3)
                    .build();

            // toIntake -> intake
            intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    tointake2Pose,
                                    intake2Pose
                            )
                    )
                    .setBrakingStrength(4)
                    .setBrakingStart(20)
                    .setLinearHeadingInterpolation(tointake2Pose.getHeading(), intake2Pose.getHeading(), 0.3)
                    .build();

            // intake -> shoot (shoot3 control)
            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    intake2Pose,
                                    scorePose
                            )
                    )
                    .setLinearHeadingInterpolation(intake2Pose.getHeading(), scorePose.getHeading(), 0.3)
                    .build();

            // shoot -> toIntake (control)
            tointake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePose,
                                    tointake3ControlPose,
                                    tointake3Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose.getHeading(), tointake3Pose.getHeading(), 0.3)
                    .build();

            // toIntake -> intake
            intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    tointake3Pose,
                                    intake3Pose
                            )
                    )
                    .setBrakingStrength(4)
                    .setBrakingStart(20)
                    .setLinearHeadingInterpolation(tointake3Pose.getHeading(), intake3Pose.getHeading(), 0.3)
                    .build();

            // intake -> shoot (shoot4 control)
            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    intake3Pose,
                                    score4ControlPose,
                                    scorePose
                            )
                    )
                    .setLinearHeadingInterpolation(intake3Pose.getHeading(), scorePose.getHeading(), 0.3)
                    .build();

            // shoot -> leave
            leave = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePose,
                                    leavePose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading(), 0.3)
                    .build();
        }
    }


    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0: // Scan
                if (!startedPath) {
                    follower.setMaxPowerScaling(1);
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
                    follower.setMaxPowerScaling(1);
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
                    follower.setMaxPowerScaling(0.9);
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
                    follower.setMaxPowerScaling(0.5);
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
                    follower.setMaxPowerScaling(0.7);
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
                    follower.setMaxPowerScaling(1);
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
                    follower.setMaxPowerScaling(0.9);
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
                    follower.setMaxPowerScaling(0.5);
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
                    follower.setMaxPowerScaling(1);
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
                    follower.setMaxPowerScaling(1);
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
                    follower.setMaxPowerScaling(0.5);
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
                    follower.setMaxPowerScaling(1);
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
                    follower.setMaxPowerScaling(1);
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