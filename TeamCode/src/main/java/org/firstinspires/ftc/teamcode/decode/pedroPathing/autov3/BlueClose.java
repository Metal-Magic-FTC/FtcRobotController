package org.firstinspires.ftc.teamcode.decode.pedroPathing.autov3;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Constants;

@Autonomous(name = "Mechanism Auto", group = "Autonomous")
public class BlueClose extends OpMode {

    // ========== MECHANISM STATE ENUMS ==========
    public enum SpindexerState {
        IDLE,
        INDEXING,
        EJECTING,
        RESET
    }

    public enum IntakeState {
        IDLE,
        INTAKING,
        OUTTAKING,
        RESET
    }

    public enum FlickerState {
        IDLE,
        FLICKING,
        RESET
    }

    public enum LauncherState {
        IDLE,
        SPINNING_UP,
        READY,
        LAUNCHING,
        RESET
    }

    public enum ShooterHoodState {
        IDLE,
        HIGH_GOAL,
        MID_GOAL,
        LOW_GOAL,
        MOVING
    }

    // ========== MECHANISM STATE VARIABLES ==========
    private SpindexerState spindexerState = SpindexerState.IDLE;
    private IntakeState intakeState = IntakeState.IDLE;
    private FlickerState flickerState = FlickerState.IDLE;
    private LauncherState launcherState = LauncherState.IDLE;
    private ShooterHoodState shooterHoodState = ShooterHoodState.IDLE;

    // ========== PATH FOLLOWING ==========
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // ========== POSES ==========
    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180));
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    // ========== PATHS ==========
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    // ========== HARDWARE (Add your actual hardware here) ==========
    // private DcMotor spindexerMotor;
    // private CRServo intakeServo;
    // private Servo flickerServo;
    // private DcMotor launcherMotor;
    // private Servo shooterHoodServo;

    /** Build all autonomous paths **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    /** Main autonomous path state machine **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start - prepare launcher and move to score
                setLauncherState(LauncherState.SPINNING_UP);
                setShooterHoodState(ShooterHoodState.HIGH_GOAL);
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: // Wait for launcher to be ready
                if (isLauncherReady() && isShooterHoodReady()) {
                    setPathState(2);
                }
                break;

            case 2: // Move to score position
                if (!follower.isBusy()) {
                    setFlickerState(FlickerState.FLICKING);
                    setPathState(3);
                }
                break;

            case 3: // Score preload
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setFlickerState(FlickerState.RESET);
                    setLauncherState(LauncherState.RESET);
                    follower.followPath(grabPickup1, true);
                    setIntakeState(IntakeState.INTAKING); // Intake while driving
                    setPathState(4);
                }
                break;

            case 4: // Grab pickup 1
                if (!follower.isBusy()) {
                    setIntakeState(IntakeState.RESET);
                    setSpindexerState(SpindexerState.INDEXING);
                    setPathState(5);
                }
                break;

            case 5: // Wait for spindexer to index
                if (isSpindexerReady()) {
                    setLauncherState(LauncherState.SPINNING_UP);
                    follower.followPath(scorePickup1, true);
                    setPathState(6);
                }
                break;

            case 6: // Move to score and wait for launcher
                if (!follower.isBusy() && isLauncherReady()) {
                    setFlickerState(FlickerState.FLICKING);
                    setPathState(7);
                }
                break;

            case 7: // Score pickup 1
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setFlickerState(FlickerState.RESET);
                    follower.followPath(grabPickup2, true);
                    setIntakeState(IntakeState.INTAKING); // Intake while driving
                    setPathState(8);
                }
                break;

            case 8: // Grab pickup 2
                if (!follower.isBusy()) {
                    setIntakeState(IntakeState.RESET);
                    setSpindexerState(SpindexerState.INDEXING);
                    setPathState(9);
                }
                break;

            case 9: // Wait for spindexer to index
                if (isSpindexerReady()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(10);
                }
                break;

            case 10: // Move to score and wait for launcher
                if (!follower.isBusy() && isLauncherReady()) {
                    setFlickerState(FlickerState.FLICKING);
                    setPathState(11);
                }
                break;

            case 11: // Score pickup 2
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setFlickerState(FlickerState.RESET);
                    follower.followPath(grabPickup3, true);
                    setIntakeState(IntakeState.INTAKING); // Intake while driving
                    setPathState(12);
                }
                break;

            case 12: // Grab pickup 3
                if (!follower.isBusy()) {
                    setIntakeState(IntakeState.RESET);
                    setSpindexerState(SpindexerState.INDEXING);
                    setPathState(13);
                }
                break;

            case 13: // Wait for spindexer to index
                if (isSpindexerReady()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(14);
                }
                break;

            case 14: // Move to score and wait for launcher
                if (!follower.isBusy() && isLauncherReady()) {
                    setFlickerState(FlickerState.FLICKING);
                    setPathState(15);
                }
                break;

            case 15: // Score pickup 3 and finish
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setFlickerState(FlickerState.RESET);
                    setLauncherState(LauncherState.RESET);
                    setPathState(-1); // Complete
                }
                break;
        }
    }

    // ========== MECHANISM CONTROL METHODS ==========

    public void setSpindexerState(SpindexerState state) {
        spindexerState = state;
        switch (state) {
            case IDLE:
                // spindexerMotor.setPower(0);
                break;
            case INDEXING:
                // spindexerMotor.setPower(0.6);
                break;
            case EJECTING:
                // spindexerMotor.setPower(-0.5);
                break;
            case RESET:
                // spindexerMotor.setPower(0);
                break;
        }
    }

    public void setIntakeState(IntakeState state) {
        intakeState = state;
        switch (state) {
            case IDLE:
            case RESET:
                // intakeServo.setPower(0);
                break;
            case INTAKING:
                // intakeServo.setPower(1.0);
                break;
            case OUTTAKING:
                // intakeServo.setPower(-1.0);
                break;
        }
    }

    public void setFlickerState(FlickerState state) {
        flickerState = state;
        switch (state) {
            case IDLE:
            case FLICKING:
                // flickerServo.setPosition(0.8);
                break;
            case RESET:
                // flickerServo.setPosition(0.2);
                break;
        }
    }

    public void setLauncherState(LauncherState state) {
        launcherState = state;
        switch (state) {
            case IDLE:
            case RESET:
                // launcherMotor.setPower(0);
                break;
            case SPINNING_UP:
            case READY:
            case LAUNCHING:
                // launcherMotor.setPower(0.95);
                break;
        }
    }

    public void setShooterHoodState(ShooterHoodState state) {
        shooterHoodState = state;
        switch (state) {
            case IDLE:
                // shooterHoodServo.setPosition(0.5);
                break;
            case HIGH_GOAL:
                // shooterHoodServo.setPosition(0.8);
                shooterHoodState = ShooterHoodState.MOVING;
                break;
            case MID_GOAL:
                // shooterHoodServo.setPosition(0.5);
                shooterHoodState = ShooterHoodState.MOVING;
                break;
            case LOW_GOAL:
                // shooterHoodServo.setPosition(0.2);
                shooterHoodState = ShooterHoodState.MOVING;
                break;
            case MOVING:
                // No action - waiting for servo to reach position
                break;
        }
    }

    // ========== MECHANISM READY CHECK METHODS ==========

    public boolean isSpindexerReady() {
        // Check if spindexer has completed indexing
        // Example: check encoder position or use timer
        if (spindexerState == SpindexerState.INDEXING) {
            if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                setSpindexerState(SpindexerState.RESET);
                return true;
            }
            return false;
        }
        return spindexerState == SpindexerState.RESET;
    }

    public boolean isLauncherReady() {
        // Check if launcher is at target velocity
        // Example: return Math.abs(launcherMotor.getVelocity() - targetVelocity) < tolerance;
        if (launcherState == LauncherState.SPINNING_UP) {
            if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                launcherState = LauncherState.READY;
                return true;
            }
            return false;
        }
        return launcherState == LauncherState.READY || launcherState == LauncherState.LAUNCHING;
    }

    public boolean isShooterHoodReady() {
        // Check if hood servo has reached target position
        // Example: return Math.abs(shooterHoodServo.getPosition() - targetPosition) < 0.05;
        if (shooterHoodState == ShooterHoodState.MOVING) {
            if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                return true;
            }
            return false;
        }
        return true;
    }

    /** Set path state and reset timer **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // ========== OPMODE LIFECYCLE METHODS ==========

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry feedback
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("---", "---");
        telemetry.addData("Spindexer", spindexerState);
        telemetry.addData("Intake", intakeState);
        telemetry.addData("Flicker", flickerState);
        telemetry.addData("Launcher", launcherState);
        telemetry.addData("Shooter Hood", shooterHoodState);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        // Initialize hardware here
        // spindexerMotor = hardwareMap.get(DcMotor.class, "spindexer");
        // intakeServo = hardwareMap.get(CRServo.class, "intake");
        // flickerServo = hardwareMap.get(Servo.class, "flicker");
        // launcherMotor = hardwareMap.get(DcMotor.class, "launcher");
        // shooterHoodServo = hardwareMap.get(Servo.class, "shooterHood");

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        // Ensure all mechanisms are stopped
        setSpindexerState(SpindexerState.IDLE);
        setIntakeState(IntakeState.RESET);
        setLauncherState(LauncherState.RESET);
    }
}