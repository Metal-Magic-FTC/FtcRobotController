package org.firstinspires.ftc.teamcode.decode.framework;

import org.firstinspires.ftc.teamcode.decode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.decode.subsystems.Spindexer;

public class ShotCycleManager {

    public enum State {
        IDLE,
        INTAKING,
        DETECTING,
        ROTATING,
        WAITING_FOR_FLYWHEEL,
        SHOOTING,
        CONFIRMING,
        COMPLETE,
        TIMEOUT
    }

    public enum Request {
        INTAKE_ONE,
        INTAKE_ALL,
        SHOOT_ONE,
        SHOOT_ALL,
        SHOOT_COLOR
    }

    private final Intake intake;
    private final Spindexer spindexer;
    private final Launcher launcher;

    private State state = State.IDLE;
    private Request currentRequest = null;
    private Spindexer.Ball targetColor = null;

    private int ballsIntaken = 0;
    private int ballsShot = 0;
    private int ballsStored = 0;
    private int currentCycle = 0;

    private long shootStartTime = 0;
    private long stateStartTime = 0;
    private static final long SHOOT_DURATION_MS = 300;
    private static final long FLYWHEEL_SETTLE_MS = 400;
    private static final long STATE_TIMEOUT_MS = 15000;

    public ShotCycleManager(Intake intake, Spindexer spindexer, Launcher launcher) {
        this.intake = intake;
        this.spindexer = spindexer;
        this.launcher = launcher;
    }

    public State getState() {
        return state;
    }

    public Request getCurrentRequest() {
        return currentRequest;
    }

    public int getBallsIntaken() {
        return ballsIntaken;
    }

    public int getBallsShot() {
        return ballsShot;
    }

    public int getBallsStored() {
        return ballsStored;
    }

    public int getCurrentCycle() {
        return currentCycle;
    }

    public boolean isBusy() {
        return state != State.IDLE && state != State.COMPLETE && state != State.TIMEOUT;
    }

    public boolean isComplete() {
        return state == State.COMPLETE;
    }

    public boolean hasTimedOut() {
        return state == State.TIMEOUT;
    }

    public void startIntakeOne() {
        start(Request.INTAKE_ONE, null);
    }

    public void startIntakeAll() {
        start(Request.INTAKE_ALL, null);
    }

    public void startShootOne() {
        start(Request.SHOOT_ONE, null);
    }

    public void startShootAll() {
        start(Request.SHOOT_ALL, null);
    }

    public void startShootColor(Spindexer.Ball color) {
        start(Request.SHOOT_COLOR, color);
    }

    private void start(Request request, Spindexer.Ball color) {
        currentRequest = request;
        targetColor = color;
        state = State.IDLE;
        stateStartTime = System.currentTimeMillis();

        switch (request) {
            case INTAKE_ONE:
            case INTAKE_ALL:
                startIntakePhase();
                break;
            case SHOOT_ONE:
            case SHOOT_ALL:
            case SHOOT_COLOR:
                startAimPhase();
                break;
        }
    }

    private void startIntakePhase() {
        state = State.INTAKING;
        stateStartTime = System.currentTimeMillis();
        intake.intake();
        spindexer.setMode(Spindexer.Mode.INTAKE);
        spindexer.startWaitingForBall();
    }

    private void startDetectPhase() {
        state = State.DETECTING;
        stateStartTime = System.currentTimeMillis();
    }

    private void startRotatePhase(int nextSlot) {
        state = State.ROTATING;
        stateStartTime = System.currentTimeMillis();
        spindexer.rotateToIndex(nextSlot);
        spindexer.startWaitingForBall();
    }

    private void startAimPhase() {
        stateStartTime = System.currentTimeMillis();
        spindexer.setMode(Spindexer.Mode.OUTTAKE);

        switch (currentRequest) {
            case SHOOT_ONE:
                int loaded = spindexer.findClosestLoaded();
                if (loaded != -1) {
                    spindexer.rotateToIndex(loaded);
                }
                break;
            case SHOOT_ALL:
                if (!spindexer.isEmpty()) {
                    int first = spindexer.findClosestLoaded();
                    if (first != -1) spindexer.rotateToIndex(first);
                }
                break;
            case SHOOT_COLOR:
                if (targetColor != null) {
                    int idx = spindexer.findClosestBall(targetColor);
                    if (idx != -1) spindexer.rotateToIndex(idx);
                }
                break;
        }

        state = State.WAITING_FOR_FLYWHEEL;
        launcher.startFlywheel();
    }

    public void update() {
        if (state == State.IDLE || state == State.COMPLETE || state == State.TIMEOUT) return;

        if (System.currentTimeMillis() - stateStartTime > STATE_TIMEOUT_MS) {
            state = State.TIMEOUT;
            stopAll();
            return;
        }

        switch (state) {
            case INTAKING:
                if (spindexer.updateIntake()) {
                    ballsIntaken++;
                    ballsStored = countStored();
                    intake.stop();

                    if (currentRequest == Request.INTAKE_ONE) {
                        state = State.COMPLETE;
                    } else {
                        int nextEmpty = spindexer.findNextEmpty();
                        if (nextEmpty != -1) {
                            startRotatePhase(nextEmpty);
                        } else {
                            state = State.COMPLETE;
                        }
                    }
                }
                break;

            case ROTATING:
                if (spindexer.isAtTarget()) {
                    startIntakePhase();
                }
                break;

            case WAITING_FOR_FLYWHEEL:
                if (System.currentTimeMillis() - stateStartTime > FLYWHEEL_SETTLE_MS) {
                    state = State.SHOOTING;
                    shootStartTime = System.currentTimeMillis();
                    launcher.startFlickMotor();
                }
                break;

            case SHOOTING:
                if (System.currentTimeMillis() - shootStartTime > SHOOT_DURATION_MS) {
                    launcher.stopFlickMotor();
                    spindexer.markCurrentEmpty();
                    ballsShot++;
                    ballsStored = countStored();

                    switch (currentRequest) {
                        case SHOOT_ONE:
                            state = State.COMPLETE;
                            break;
                        case SHOOT_ALL:
                            int next = spindexer.findClosestLoaded();
                            if (next != -1) {
                                spindexer.rotateToIndex(next);
                                stateStartTime = System.currentTimeMillis();
                                state = State.WAITING_FOR_FLYWHEEL;
                                launcher.startFlywheel();
                            } else {
                                state = State.COMPLETE;
                            }
                            break;
                        case SHOOT_COLOR:
                            int nextColored = spindexer.findClosestBall(targetColor);
                            if (nextColored != -1) {
                                spindexer.rotateToIndex(nextColored);
                                stateStartTime = System.currentTimeMillis();
                                state = State.WAITING_FOR_FLYWHEEL;
                            } else {
                                state = State.COMPLETE;
                            }
                            break;
                    }
                }
                break;
        }
    }

    private int countStored() {
        int count = 0;
        for (int i = 0; i < 3; i++) {
            if (spindexer.getSlot(i) != Spindexer.Ball.EMPTY) count++;
        }
        return count;
    }

    public void reset() {
        state = State.IDLE;
        currentRequest = null;
        targetColor = null;
        ballsIntaken = 0;
        ballsShot = 0;
        ballsStored = 0;
        currentCycle = 0;
    }

    public void stopAll() {
        intake.stop();
        launcher.stopAll();
        spindexer.cancelSweep();
        spindexer.stopWaitingForBall();
    }

    public String getStatusString() {
        if (state == State.IDLE) return "Idle";
        return String.format("%s | Intaken: %d | Stored: %d | Shot: %d | Slots: %s",
                state.name(), ballsIntaken, ballsStored, ballsShot, spindexer.getSlotsString());
    }
}
