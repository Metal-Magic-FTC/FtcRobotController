package org.firstinspires.ftc.teamcode.biobuzz.comp1.vision;

/**
 * Kalman filter that fuses odometry (Pinpoint) with Limelight pose fixes, one independent filter per axis
 * (x, y, heading). The pose itself lives in the Pedro follower; this class only tracks HOW SURE we are of it
 * (the variance P of each axis) and decides how much of each vision error to apply (the Kalman gain).
 *
 *   PREDICT (every loop):  P += growth from how far the robot drove / turned on odometry alone, plus a little per second
 *   UPDATE  (each vision frame):  R = how noisy THIS frame is (tag distance, number of tags)
 *                                 K = P / (P + R)          -> fraction of the vision error to apply (0..1)
 *                                 P = (1 - K) * P          -> more sure after the fix
 *   GATE: a frame whose error is more than GATE_SIGMA standard deviations from what P + R expects is rejected.
 *
 * Long drives without tags -> P grows -> the next fix is trusted a lot. Right after a good fix -> P is small ->
 * a single noisy far frame barely moves the pose. Units: inches and degrees (variances in in^2 and deg^2).
 */
public class PoseKalman {

    // ======================= ODOMETRY NOISE (process) =======================
    /** Position variance added per inch driven (in^2 per in). 0.01 ~ 1 in of drift (1 sigma) per 100 in driven. */
    public static double ODO_XY_VAR_PER_IN = 0.01;
    /** Position variance added per second, even standing still (wheel slip, being bumped). */
    public static double ODO_XY_VAR_PER_SEC = 0.02;
    /** Heading variance added per degree turned (deg^2 per deg). 0.004 ~ 1.2 deg (1 sigma) per full turn. */
    public static double ODO_HEADING_VAR_PER_DEG = 0.004;
    /**
     * Heading variance added per second. Set on the generous side on purpose: it keeps the filter from getting so
     * sure of its heading that vision heading fixes stop mattering. Raise it for bigger heading corrections per frame.
     */
    public static double ODO_HEADING_VAR_PER_SEC = 0.05;
    /** Uncertainty never grows past this (1 sigma): keeps one fix from being trusted 100% after a long blind drive. */
    public static double MAX_XY_SIGMA_IN = 12.0;
    public static double MAX_HEADING_SIGMA_DEG = 10.0;

    // ======================= VISION NOISE (measurement) =======================
    /** Position noise of one tag at zero range (1 sigma), plus this much per inch of range. MEASURE / TUNE. */
    public static double VISION_XY_SIGMA_BASE_IN = 0.75;
    public static double VISION_XY_SIGMA_PER_IN = 0.02;
    /** Heading noise of one tag pair at zero range, plus per inch of range (degrees, 1 sigma). */
    public static double VISION_HEADING_SIGMA_BASE_DEG = 1.0;
    public static double VISION_HEADING_SIGMA_PER_IN = 0.02;

    // ======================= GATE =======================
    /** Reject a frame whose error is beyond this many standard deviations (sqrt(P + R)). */
    public static double GATE_SIGMA = 3.0;

    // ======================= START =======================
    /** Uncertainty right after the first lock / a snap (1 sigma). */
    public static double LOCK_XY_SIGMA_IN = 2.0;
    public static double LOCK_HEADING_SIGMA_DEG = 2.0;

    private double pX, pY, pH; // variances: in^2, in^2, deg^2
    private double lastGainXY = Double.NaN, lastGainH = Double.NaN;
    private int rejected = 0;

    public PoseKalman() {
        reset();
    }

    /** Call after the first lock or a snap: the pose was just set from vision. */
    public void reset() {
        pX = pY = LOCK_XY_SIGMA_IN * LOCK_XY_SIGMA_IN;
        pH = LOCK_HEADING_SIGMA_DEG * LOCK_HEADING_SIGMA_DEG;
    }

    /**
     * PREDICT: the robot moved on odometry alone.
     *
     * @param movedIn    distance driven since the last call (inches)
     * @param turnedDeg  rotation since the last call (degrees, any sign)
     * @param dtSec      time since the last call
     */
    public void predict(double movedIn, double turnedDeg, double dtSec) {
        double qXY = ODO_XY_VAR_PER_IN * Math.abs(movedIn) + ODO_XY_VAR_PER_SEC * dtSec;
        double qH = ODO_HEADING_VAR_PER_DEG * Math.abs(turnedDeg) + ODO_HEADING_VAR_PER_SEC * dtSec;
        double maxXY = MAX_XY_SIGMA_IN * MAX_XY_SIGMA_IN, maxH = MAX_HEADING_SIGMA_DEG * MAX_HEADING_SIGMA_DEG;
        pX = Math.min(pX + qXY, maxXY);
        pY = Math.min(pY + qXY, maxXY);
        pH = Math.min(pH + qH, maxH);
    }

    /** Vision position noise (1 sigma, inches) for a frame: worse with range, better with more tags. */
    public static double visionXYSigma(double rangeIn, int tagCount) {
        return (VISION_XY_SIGMA_BASE_IN + VISION_XY_SIGMA_PER_IN * rangeIn) / Math.sqrt(Math.max(1, tagCount));
    }

    /** Vision heading noise (1 sigma, degrees) for a frame. */
    public static double visionHeadingSigma(double rangeIn, int tagCount) {
        return (VISION_HEADING_SIGMA_BASE_DEG + VISION_HEADING_SIGMA_PER_IN * rangeIn)
                / Math.sqrt(Math.max(1, tagCount / 2.0));
    }

    /**
     * UPDATE x/y with one vision frame.
     *
     * @return {dx, dy} to apply to the pose, or null if the frame was rejected by the gate
     */
    public double[] updateXY(double errX, double errY, double sigmaIn) {
        double r = sigmaIn * sigmaIn;
        // Gate on the combined (2D) error against the larger of the two axes' expected spread.
        double expected = Math.max(pX, pY) + r;
        if (errX * errX + errY * errY > GATE_SIGMA * GATE_SIGMA * expected) {
            rejected++;
            return null;
        }
        double kX = pX / (pX + r), kY = pY / (pY + r);
        pX *= (1 - kX);
        pY *= (1 - kY);
        lastGainXY = 0.5 * (kX + kY);
        return new double[]{kX * errX, kY * errY};
    }

    /**
     * UPDATE heading with one vision frame (degrees).
     *
     * @return heading change to apply (degrees), or NaN if the frame was rejected by the gate
     */
    public double updateHeading(double errDeg, double sigmaDeg) {
        double r = sigmaDeg * sigmaDeg;
        if (errDeg * errDeg > GATE_SIGMA * GATE_SIGMA * (pH + r)) {
            rejected++;
            return Double.NaN;
        }
        double k = pH / (pH + r);
        pH *= (1 - k);
        lastGainH = k;
        return k * errDeg;
    }

    public double xySigmaIn() {
        return Math.sqrt(Math.max(pX, pY));
    }

    public double headingSigmaDeg() {
        return Math.sqrt(pH);
    }

    public double lastGainXY() {
        return lastGainXY;
    }

    public double lastGainHeading() {
        return lastGainH;
    }

    public int rejectedFrames() {
        return rejected;
    }
}
