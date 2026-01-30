package org.firstinspires.ftc.teamcode.decode.pedroPathing.optStatesAutos;

/**
 * Simple loop timer for measuring FTC OpMode loop performance.
 * Tracks average, min, max, and recent loop times.
 */
public class LoopTimer {
    private long lastLoopTime;
    private long totalTime;
    private long loopCount;
    private long minLoopTime;
    private long maxLoopTime;

    public LoopTimer() {
        reset();
    }

    /**
     * Call this at the START of each loop iteration.
     */
    public void update() {
        long currentTime = System.nanoTime();

        if (lastLoopTime != 0) {
            long loopTime = currentTime - lastLoopTime;
            totalTime += loopTime;
            loopCount++;

            if (loopTime < minLoopTime) minLoopTime = loopTime;
            if (loopTime > maxLoopTime) maxLoopTime = loopTime;
        }

        lastLoopTime = currentTime;
    }

    /**
     * Gets the average loop time in milliseconds.
     */
    public double getAverageMs() {
        if (loopCount == 0) return 0;
        return (totalTime / (double) loopCount) / 1_000_000.0;
    }

    /**
     * Gets the average loops per second (Hz).
     */
    public double getAverageHz() {
        double avgMs = getAverageMs();
        if (avgMs == 0) return 0;
        return 1000.0 / avgMs;
    }

    /**
     * Gets the minimum loop time in milliseconds.
     */
    public double getMinMs() {
        if (loopCount == 0) return 0;
        return minLoopTime / 1_000_000.0;
    }

    /**
     * Gets the maximum loop time in milliseconds.
     */
    public double getMaxMs() {
        if (loopCount == 0) return 0;
        return maxLoopTime / 1_000_000.0;
    }

    /**
     * Gets the total number of loops counted.
     */
    public long getLoopCount() {
        return loopCount;
    }

    /**
     * Resets all timing data.
     */
    public void reset() {
        lastLoopTime = 0;
        totalTime = 0;
        loopCount = 0;
        minLoopTime = Long.MAX_VALUE;
        maxLoopTime = 0;
    }

    /**
     * Gets a formatted string for telemetry.
     */
    public String getTelemetryString() {
        return String.format("Avg: %.1fms (%.0fHz) | Min: %.1fms | Max: %.1fms | Loops: %d",
                getAverageMs(), getAverageHz(), getMinMs(), getMaxMs(), loopCount);
    }
}