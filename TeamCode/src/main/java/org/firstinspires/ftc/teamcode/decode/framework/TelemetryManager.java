package org.firstinspires.ftc.teamcode.decode.framework;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

public class TelemetryManager {

    public enum Level {
        COMPETITION(0),
        INFO(1),
        DEBUG(2);

        final int priority;
        Level(int priority) { this.priority = priority; }
    }

    public enum Category {
        DRIVE,
        INTAKE,
        LAUNCHER,
        SPINDEXER,
        VISION,
        CYCLE,
        GAME,
        AUTONOMOUS,
        SYSTEM
    }

    private Level currentLevel = Level.INFO;
    private final Map<String, String> dataLines = new LinkedHashMap<>();
    private final Map<String, String> infoLines = new LinkedHashMap<>();

    private long lastCycleTime = 0;
    private double smoothedHz = 0;
    private int totalCycles = 0;

    public void setLevel(Level level) {
        this.currentLevel = level;
    }

    public Level getLevel() {
        return currentLevel;
    }

    public void addData(Category category, String key, Object value) {
        addData(category, key, value, Level.INFO);
    }

    public void addData(Category category, String key, Object value, Level level) {
        if (level.priority > currentLevel.priority) return;
        String prefix = category == null ? "" : "[" + category.name() + "] ";
        dataLines.put(prefix + key, String.valueOf(value));
    }

    public void addInfo(String key, String value, Level level) {
        if (level.priority > currentLevel.priority) return;
        infoLines.put(key, value);
    }

    public void addInfo(String key, String value) {
        addInfo(key, value, Level.INFO);
    }

    public void flush(Telemetry telemetry) {
        long now = System.nanoTime();
        if (lastCycleTime != 0) {
            double dt = (now - lastCycleTime) / 1e9;
            if (dt > 0) {
                double instantHz = 1.0 / dt;
                smoothedHz = smoothedHz == 0 ? instantHz : smoothedHz * 0.8 + instantHz * 0.2;
            }
        }
        lastCycleTime = now;
        totalCycles++;

        if (currentLevel.priority >= Level.COMPETITION.priority) {
            telemetry.addData("LOOP Hz", String.format("%.1f", smoothedHz));
        }

        for (Map.Entry<String, String> entry : dataLines.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }

        for (Map.Entry<String, String> entry : infoLines.entrySet()) {
            telemetry.addLine(entry.getKey() + ": " + entry.getValue());
        }

        telemetry.update();
        dataLines.clear();
        infoLines.clear();
    }

    public TelemetryPacket newDashboardPacket() {
        return new TelemetryPacket();
    }

    public void sendDashboardPacket(TelemetryPacket packet) {
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public double getLoopHz() {
        return smoothedHz;
    }

    public int getTotalCycles() {
        return totalCycles;
    }
}
