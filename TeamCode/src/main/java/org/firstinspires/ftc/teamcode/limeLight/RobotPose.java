package org.firstinspires.ftc.teamcode.limeLight;

/**
 * Represents a robot's pose (position and orientation) on the FTC field.
 *
 * Coordinate System:
 * - X: Distance along the field length (positive forward)
 * - Y: Distance along the field width (positive left)
 * - Heading: Robot orientation in degrees (0 = forward, counterclockwise positive)
 *
 * All distances are in meters unless otherwise specified.
 */
public class RobotPose {

    /** X coordinate in meters (forward/backward on field) */
    private final double x;

    /** Y coordinate in meters (left/right on field) */
    private final double y;

    /** Heading in degrees (0 = forward, counterclockwise positive) */
    private final double heading;

    /** Timestamp when this pose was measured (in milliseconds) */
    private final long timestamp;

    /** Confidence score (0.0 to 1.0) - higher is better */
    private final double confidence;

    /**
     * Creates a new RobotPose with the current system time.
     *
     * @param x X coordinate in meters
     * @param y Y coordinate in meters
     * @param heading Heading in degrees
     * @param confidence Confidence score (0.0 to 1.0)
     */
    public RobotPose(double x, double y, double heading, double confidence) {
        this(x, y, heading, confidence, System.currentTimeMillis());
    }

    /**
     * Creates a new RobotPose with a specified timestamp.
     *
     * @param x X coordinate in meters
     * @param y Y coordinate in meters
     * @param heading Heading in degrees
     * @param confidence Confidence score (0.0 to 1.0)
     * @param timestamp Timestamp in milliseconds
     */
    public RobotPose(double x, double y, double heading, double confidence, long timestamp) {
        this.x = x;
        this.y = y;
        this.heading = normalizeHeading(heading);
        this.confidence = Math.max(0.0, Math.min(1.0, confidence)); // Clamp to [0, 1]
        this.timestamp = timestamp;
    }

    /**
     * Normalizes heading to be within [-180, 180] degrees.
     */
    private double normalizeHeading(double heading) {
        while (heading > 180.0) heading -= 360.0;
        while (heading < -180.0) heading += 360.0;
        return heading;
    }

    // Getters

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public double getConfidence() {
        return confidence;
    }

    /**
     * Gets the age of this pose measurement in milliseconds.
     */
    public long getAgeMs() {
        return System.currentTimeMillis() - timestamp;
    }

    /**
     * Calculates the Euclidean distance to another pose.
     */
    public double distanceTo(RobotPose other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculates the angular difference to another pose's heading.
     * Result is in range [-180, 180] degrees.
     */
    public double headingDifference(RobotPose other) {
        return normalizeHeading(this.heading - other.heading);
    }

    /**
     * Converts position to inches for display.
     */
    public double getXInches() {
        return x * 39.3701; // meters to inches
    }

    public double getYInches() {
        return y * 39.3701; // meters to inches
    }

    @Override
    public String toString() {
        return String.format("Pose(x=%.3fm, y=%.3fm, heading=%.1f°, conf=%.2f)",
                x, y, heading, confidence);
    }

    /**
     * Returns a formatted string with positions in inches.
     */
    public String toStringInches() {
        return String.format("Pose(x=%.1fin, y=%.1fin, heading=%.1f°, conf=%.2f)",
                getXInches(), getYInches(), heading, confidence);
    }
}
