package org.firstinspires.ftc.teamcode.decode.framework;

import org.firstinspires.ftc.teamcode.decode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.decode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.decode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.decode.util.MotifDecoder;

import java.util.ArrayList;
import java.util.List;

public class RobotCoordinator {

    private final Intake intake;
    private final Launcher launcher;
    private final Spindexer spindexer;
    private final ShotCycleManager cycleManager;
    private final TelemetryManager telemetryManager;

    private boolean isActive = false;

    public RobotCoordinator(Intake intake, Launcher launcher, Spindexer spindexer) {
        this.intake = intake;
        this.launcher = launcher;
        this.spindexer = spindexer;
        this.cycleManager = new ShotCycleManager(intake, spindexer, launcher);
        this.telemetryManager = null;
    }

    public RobotCoordinator(Intake intake, Launcher launcher, Spindexer spindexer, TelemetryManager telemetryManager) {
        this.intake = intake;
        this.launcher = launcher;
        this.spindexer = spindexer;
        this.cycleManager = new ShotCycleManager(intake, spindexer, launcher);
        this.telemetryManager = telemetryManager;
    }

    public ShotCycleManager getCycleManager() {
        return cycleManager;
    }

    // ==================== HIGH-LEVEL OPERATIONS ====================

    public void startPatternShot(MotifDecoder.Motif motif) {
        isActive = true;
        if (spindexer.isEmpty()) {
            spindexer.setMode(Spindexer.Mode.INTAKE);
            cycleManager.startIntakeAll();
            log("Pattern shot started, intaking balls");
        } else {
            int startIdx = MotifDecoder.findBestStartIndex(
                    new Spindexer.Ball[]{spindexer.getSlot(0), spindexer.getSlot(1), spindexer.getSlot(2)},
                    motif
            );
            spindexer.rotateToIndex(startIdx);
            cycleManager.startShootColor(motif.getSlot(startIdx));
            log("Pattern shot started, aiming slot " + startIdx);
        }
    }

    public void startIntakeCycle() {
        isActive = true;
        spindexer.setMode(Spindexer.Mode.INTAKE);
        cycleManager.startIntakeAll();
        log("Intake cycle started");
    }

    public void startIntakeOne() {
        isActive = true;
        spindexer.setMode(Spindexer.Mode.INTAKE);
        cycleManager.startIntakeOne();
        log("Intake one started");
    }

    public void startShootAll() {
        isActive = true;
        if (!spindexer.isEmpty()) {
            spindexer.setMode(Spindexer.Mode.OUTTAKE);
            cycleManager.startShootAll();
            log("Shoot all started with " + countBalls() + " balls");
        }
    }

    public void startShootColor(Spindexer.Ball color) {
        isActive = true;
        if (spindexer.contains(color)) {
            spindexer.setMode(Spindexer.Mode.OUTTAKE);
            cycleManager.startShootColor(color);
            log("Shoot color " + color + " started");
        }
    }

    public void startLauncherAndHold(double velocity) {
        launcher.startFlywheel(velocity);
        log("Launcher started at " + velocity);
    }

    public void stopLauncher() {
        launcher.stopFlywheel();
        log("Launcher stopped");
    }

    // ==================== GATE CONTROL ====================

    private long gateHoldStartTime = 0;
    private boolean gateHolding = false;
    private static final long GATE_HOLD_MS = 1500;

    public void startOpenGate() {
        gateHolding = true;
        gateHoldStartTime = System.currentTimeMillis();
        log("Gate opening");
    }

    public boolean isGateOpenComplete() {
        if (!gateHolding) return true;
        if (System.currentTimeMillis() - gateHoldStartTime >= GATE_HOLD_MS) {
            gateHolding = false;
            log("Gate closed");
            return true;
        }
        return false;
    }

    public void cancelGate() {
        gateHolding = false;
    }

    // ==================== UPDATE LOOP ====================

    public void update() {
        cycleManager.update();
        telemetryIfPresent();
    }

    public boolean isBusy() {
        return cycleManager.isBusy() || gateHolding;
    }

    public void stopAll() {
        isActive = false;
        cycleManager.stopAll();
        cancelGate();
        log("All systems stopped");
    }

    // ==================== QUERIES ====================

    public int countBalls() {
        int count = 0;
        for (int i = 0; i < 3; i++) {
            if (spindexer.getSlot(i) != Spindexer.Ball.EMPTY) count++;
        }
        return count;
    }

    public boolean isFull() {
        return spindexer.isFull();
    }

    public boolean isEmpty() {
        return spindexer.isEmpty();
    }

    public List<Spindexer.Ball> getStoredBalls() {
        List<Spindexer.Ball> balls = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            Spindexer.Ball b = spindexer.getSlot(i);
            if (b != Spindexer.Ball.EMPTY) balls.add(b);
        }
        return balls;
    }

    // ==================== TELEMETRY ====================

    private void log(String message) {
        if (telemetryManager != null) {
            telemetryManager.addInfo("[Coordinator]", message, TelemetryManager.Level.INFO);
        }
    }

    private void telemetryIfPresent() {
        if (telemetryManager == null) return;

        telemetryManager.addData(TelemetryManager.Category.CYCLE, "State", cycleManager.getState().name());
        telemetryManager.addData(TelemetryManager.Category.CYCLE, "Intaken", cycleManager.getBallsIntaken());
        telemetryManager.addData(TelemetryManager.Category.CYCLE, "Shot", cycleManager.getBallsShot());
        telemetryManager.addData(TelemetryManager.Category.CYCLE, "Stored", cycleManager.getBallsStored());
        telemetryManager.addData(TelemetryManager.Category.SPINDEXER, "Slots", spindexer.getSlotsString());
        telemetryManager.addData(TelemetryManager.Category.SPINDEXER, "Index", spindexer.getCurrentIndex());
        telemetryManager.addData(TelemetryManager.Category.LAUNCHER, "Vel", String.format("%.0f", launcher.getFlywheelVelocity()));
        telemetryManager.addData(TelemetryManager.Category.LAUNCHER, "Hood", String.format("%.3f", launcher.getHoodPosition()));
        telemetryManager.addData(TelemetryManager.Category.INTAKE, "Power", String.format("%.2f", intake.getPower()));
    }
}
