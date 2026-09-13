package org.firstinspires.ftc.teamcode.biobuzz.comp1.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/**
 * Shared BIOBUZZ HIVE AprilTag geometry + math (all numbers from the V1 Competition Manual, section 9).
 *
 * FIELD FRAME (Pedro Pathing, inches)
 *   Looking at the field from the AUDIENCE:
 *   - origin (0,0) = bottom-left corner (red alliance wall / audience wall corner)
 *   - +X to the right (toward the blue alliance wall), +Y away from the audience
 *   - heading 0 = facing +X, counter-clockwise positive
 *   - field center = (72, 72)
 *
 * HIVE (manual 9.6, Fig 9-8 .. 9-10)
 *   - Pivot axis runs left/right (parallel to X), 43.95 in above the tiles, at the field center line Y = 72.
 *   - Red HIVE center X = 72 - 25.5/2, blue HIVE center X = 72 + 25.5/2.
 *   - Each HIVE has an AUDIENCE cell (low Y side) and a SCORING cell (high Y side), 42.91 in end to end,
 *     12.04 in deep cells, stable tilt = 30 degrees (either way).
 *
 * APRILTAGS (manual 9.9, Fig 9-15 .. 9-17)
 *   - 36h11, 3.25 in (82.55 mm). 4 tags per cell, stuck on the BOTTOM of the cell facing the tiles.
 *   - Tag centers are 2.75 in and 6.5 in either side of the cluster centerline (which sits on the HIVE arm).
 *   - Cluster centerline is 9.938 - 2.75 = 7.188 in from the outer ("front") edge of the cell, so it is
 *     42.91/2 - 7.188 = 14.27 in from the pivot along the arm.
 *   - The cell floor (tag surface) sits ~1.75 in below the pivot line when the HIVE is level (measured off
 *     Fig 9-9; it reproduces the Fig 9-10 tilted drawing to within ~1 in).
 *   - IDs (left -> right as seen from the audience, Fig 9-17):
 *       red scoring 33 32 31 30 | blue scoring 45 44 43 42
 *       red audience 34 35 36 37 | blue audience 38 39 40 41
 *
 * TILT SIGN
 *   tilt > 0  => SCORING cell is UP (audience cell down)
 *   tilt < 0  => AUDIENCE cell is UP (scoring cell down)
 *
 * HOW TILT IS MEASURED
 *   The cluster is a straight line of tags parallel to the pivot, so the only thing tilt changes is where the
 *   tag sits in the Y/Z plane. Up-cell tags sit at ~49.6 in, down-cell tags at ~35.3 in, level at ~42.2 in.
 *   We take the tag's 3D position from the Limelight (translation is very reliable, the tag's own solved
 *   rotation is not at these oblique viewing angles), compute its height above the tiles using the camera
 *   mount, then solve  height - pivot = armSign * r * sin(tilt) - h * cos(tilt)  for tilt.
 */
public final class HiveTagGeometry {

    private HiveTagGeometry() {}

    // ======================= CAMERA MOUNT (MEASURE THESE ON YOUR ROBOT) =======================
    /** Limelight pipeline index that is set up for AprilTags (36h11, tag size 82.55 mm, "Full 3D" on). */
    public static final int APRILTAG_PIPELINE = 1;
    /** Lens position relative to robot center, inches. Forward is robot front, left is robot left. */
    public static final double CAMERA_FORWARD_IN = 7.0;
    public static final double CAMERA_LEFT_IN = 0.0;
    /** Lens height above the tiles, inches. Tilt detection depends directly on this number. */
    public static final double CAMERA_HEIGHT_IN = 10.0;
    /** 0 = looking straight ahead parallel to the ground. Positive = tilted up. */
    public static final double CAMERA_PITCH_DEG = 0.0;
    /** 0 = pointing out the robot front. Positive = rotated toward robot left. */
    public static final double CAMERA_YAW_DEG = 0.0;

    // ======================= FIELD / HIVE NUMBERS (MANUAL) =======================
    public static final double FIELD_CENTER_X = 72.0;
    public static final double FIELD_CENTER_Y = 72.0;

    public static final double PIVOT_HEIGHT_IN = 43.95;
    public static final double PIVOT_Y_IN = 72.0;
    public static final double HIVE_CENTER_TO_CENTER_IN = 25.5;
    public static final double RED_HIVE_X_IN = FIELD_CENTER_X - HIVE_CENTER_TO_CENTER_IN / 2.0;
    public static final double BLUE_HIVE_X_IN = FIELD_CENTER_X + HIVE_CENTER_TO_CENTER_IN / 2.0;

    public static final double HIVE_LENGTH_IN = 42.91;
    public static final double REF_HOLE_FROM_CELL_FRONT_IN = 9.938;
    public static final double CLUSTER_ABOVE_REF_HOLES_IN = 2.75;
    /** Distance from pivot to tag cluster centerline, along the arm (level HIVE). */
    public static final double TAG_ARM_DISTANCE_IN =
            HIVE_LENGTH_IN / 2.0 - (REF_HOLE_FROM_CELL_FRONT_IN - CLUSTER_ABOVE_REF_HOLES_IN);
    /** Tag surface below the pivot line (level HIVE). */
    public static final double TAG_BELOW_PIVOT_IN = 1.75;

    public static final double STABLE_TILT_DEG = 30.0;
    /** |tilt| at or above this is treated as resting in a stable position; below it is TRANSITION. */
    public static final double STABLE_MIN_DEG = 20.0;

    /** Tag center offsets along X from the cluster centerline, left -> right as seen from the audience. */
    private static final double[] TAG_X_OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};

    /** Reject detections whose height makes no geometric sense (bad camera constants / bad solve). */
    private static final double HEIGHT_SLACK_IN = 4.0;
    private static final double MAX_STALENESS_MS = 150;

    // ======================= TYPES =======================
    public enum Hive { RED, BLUE }

    public enum Cell { AUDIENCE, SCORING }

    public enum TiltState {
        SCORING_UP("SCORING cell UP (audience cell down)"),
        AUDIENCE_UP("AUDIENCE cell UP (scoring cell down)"),
        TRANSITION("IN TRANSITION"),
        UNKNOWN("not seen");

        public final String label;

        TiltState(String label) {
            this.label = label;
        }
    }

    /** Where a tag ID lives on the field. */
    public static final class TagInfo {
        public final int id;
        public final Hive hive;
        public final Cell cell;
        /** +1 for the scoring cell (+Y side of the pivot), -1 for the audience cell. */
        public final int armSign;
        public final double fieldX;

        TagInfo(int id, Hive hive, Cell cell, double xOffset) {
            this.id = id;
            this.hive = hive;
            this.cell = cell;
            this.armSign = cell == Cell.SCORING ? 1 : -1;
            this.fieldX = (hive == Hive.RED ? RED_HIVE_X_IN : BLUE_HIVE_X_IN) + xOffset;
        }
    }

    /** One tag seen this frame, already converted into robot coordinates. */
    public static final class TagObservation {
        public final TagInfo info;
        /** Tag center relative to robot center, inches (forward, left) and height above the tiles. */
        public final double forward, left, height;
        public final double rangeIn;
        /** HIVE tilt implied by this single tag's height, degrees. */
        public final double tiltDeg;

        TagObservation(TagInfo info, double forward, double left, double height, double rangeIn) {
            this.info = info;
            this.forward = forward;
            this.left = left;
            this.height = height;
            this.rangeIn = rangeIn;
            this.tiltDeg = Math.toDegrees(solveTiltFromHeight(height, info.armSign));
        }
    }

    /** Combined tilt reading for one HIVE from every tag of that HIVE seen this frame. */
    public static final class HiveTilt {
        public final Hive hive;
        public final double tiltDeg;
        public final int tagCount;
        public final TiltState state;

        HiveTilt(Hive hive, double tiltDeg, int tagCount) {
            this.hive = hive;
            this.tiltDeg = tiltDeg;
            this.tagCount = tagCount;
            this.state = tagCount == 0 ? TiltState.UNKNOWN : classify(tiltDeg);
        }
    }

    // ======================= LOOKUP =======================
    public static TagInfo lookup(int id) {
        if (id >= 30 && id <= 33) return new TagInfo(id, Hive.RED, Cell.SCORING, TAG_X_OFFSETS_IN[3 - (id - 30)]);
        if (id >= 34 && id <= 37) return new TagInfo(id, Hive.RED, Cell.AUDIENCE, TAG_X_OFFSETS_IN[id - 34]);
        if (id >= 38 && id <= 41) return new TagInfo(id, Hive.BLUE, Cell.AUDIENCE, TAG_X_OFFSETS_IN[id - 38]);
        if (id >= 42 && id <= 45) return new TagInfo(id, Hive.BLUE, Cell.SCORING, TAG_X_OFFSETS_IN[3 - (id - 42)]);
        return null;
    }

    // ======================= VISION -> ROBOT FRAME =======================
    /**
     * Reads every HIVE tag from a Limelight result and converts each into robot coordinates.
     * Limelight camera space: +x right, +y down, +z out of the lens (meters).
     */
    public static List<TagObservation> readTags(LLResult result) {
        List<TagObservation> out = new ArrayList<>();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) return out;

        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        double yaw = Math.toRadians(CAMERA_YAW_DEG);

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            TagInfo info = lookup(tag.getFiducialId());
            if (info == null) continue;
            Pose3D camPose = tag.getTargetPoseCameraSpace();
            if (camPose == null) continue;

            Position p = camPose.getPosition().toUnit(DistanceUnit.INCH);
            if (p.z <= 0) continue; // no 3D solve (tag behind lens / not computed)

            // Undo the camera pitch: forward/up in a level frame attached to the camera
            double levelForward = p.z * Math.cos(pitch) + p.y * Math.sin(pitch);
            double levelUp = p.z * Math.sin(pitch) - p.y * Math.cos(pitch);
            double levelLeft = -p.x;

            // Undo the camera yaw and add the mount offset
            double forward = CAMERA_FORWARD_IN + levelForward * Math.cos(yaw) - levelLeft * Math.sin(yaw);
            double left = CAMERA_LEFT_IN + levelForward * Math.sin(yaw) + levelLeft * Math.cos(yaw);
            double height = CAMERA_HEIGHT_IN + levelUp;

            double r = Math.hypot(TAG_ARM_DISTANCE_IN, TAG_BELOW_PIVOT_IN);
            if (Math.abs(height - PIVOT_HEIGHT_IN) > r + HEIGHT_SLACK_IN) continue;

            out.add(new TagObservation(info, forward, left, height, Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)));
        }
        return out;
    }

    // ======================= TILT =======================
    /**
     * Tag height above tiles for a HIVE tilt:  z = pivot + armSign * r * sin(t) - h * cos(t)
     * Rewritten as  (z - pivot) = R * sin(armSign * t - alpha)  with R = hypot(r, h), alpha = atan2(h, r),
     * which gives  t = armSign * (asin((z - pivot) / R) + alpha).
     */
    public static double solveTiltFromHeight(double tagHeight, int armSign) {
        double r = TAG_ARM_DISTANCE_IN;
        double h = TAG_BELOW_PIVOT_IN;
        double bigR = Math.hypot(r, h);
        double alpha = Math.atan2(h, r);
        double s = Range.clip((tagHeight - PIVOT_HEIGHT_IN) / bigR, -1.0, 1.0);
        return armSign * (Math.asin(s) + alpha);
    }

    public static TiltState classify(double tiltDeg) {
        if (tiltDeg >= STABLE_MIN_DEG) return TiltState.SCORING_UP;
        if (tiltDeg <= -STABLE_MIN_DEG) return TiltState.AUDIENCE_UP;
        return TiltState.TRANSITION;
    }

    /** Averages the tilt implied by every tag of one HIVE (both cells agree because the HIVE is rigid). */
    public static HiveTilt measureHive(List<TagObservation> tags, Hive hive) {
        double sum = 0;
        int n = 0;
        for (TagObservation t : tags) {
            if (t.info.hive != hive) continue;
            sum += t.tiltDeg;
            n++;
        }
        return new HiveTilt(hive, n == 0 ? 0 : sum / n, n);
    }

    /**
     * The tilt to use when placing tags on the field: snap to exactly +/-30 when resting (the manual geometry is
     * more accurate than our height estimate), otherwise use the measured angle while it swings.
     */
    public static double tiltForLocalizationDeg(HiveTilt tilt) {
        switch (tilt.state) {
            case SCORING_UP:
                return STABLE_TILT_DEG;
            case AUDIENCE_UP:
                return -STABLE_TILT_DEG;
            default:
                return tilt.tiltDeg;
        }
    }

    /** Tag center field position {x, y, z} for a given HIVE tilt (rotation about the pivot axis). */
    public static double[] tagFieldPosition(TagInfo info, double tiltRad) {
        double armY = info.armSign * TAG_ARM_DISTANCE_IN;
        double armZ = -TAG_BELOW_PIVOT_IN;
        double y = PIVOT_Y_IN + armY * Math.cos(tiltRad) - armZ * Math.sin(tiltRad);
        double z = PIVOT_HEIGHT_IN + armY * Math.sin(tiltRad) + armZ * Math.cos(tiltRad);
        return new double[]{info.fieldX, y, z};
    }

    // ======================= LOCALIZATION =======================
    /**
     * Robot center {x, y} on the field from one tag:  robot = tagField - R(heading) * tagInRobot
     */
    public static double[] robotPositionFromTag(TagObservation obs, double tiltRad, double headingRad) {
        double[] tag = tagFieldPosition(obs.info, tiltRad);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double x = tag[0] - (obs.forward * cos - obs.left * sin);
        double y = tag[1] - (obs.forward * sin + obs.left * cos);
        return new double[]{x, y};
    }

    /**
     * Robot heading from two tags on the same cell. Tags on a cell lie on a line parallel to field X, and tilting
     * the HIVE does not move them along X, so the direction between them is known no matter the tilt.
     * Returns NaN if no usable pair is visible.
     */
    public static double headingFromTagPair(List<TagObservation> tags) {
        TagObservation bestA = null, bestB = null;
        double bestSpan = 0;
        for (TagObservation a : tags) {
            for (TagObservation b : tags) {
                if (a.info.hive != b.info.hive || a.info.cell != b.info.cell) continue;
                double span = b.info.fieldX - a.info.fieldX;
                if (span > bestSpan) {
                    bestSpan = span;
                    bestA = a;
                    bestB = b;
                }
            }
        }
        if (bestA == null || bestSpan < 5.0) return Double.NaN;
        // Field direction A->B is +X (angle 0); robot-frame direction is atan2(dLeft, dForward)
        double robotAngle = Math.atan2(bestB.left - bestA.left, bestB.forward - bestA.forward);
        return normalizeAngle(-robotAngle);
    }

    public static double headingToFieldCenter(double x, double y) {
        return Math.atan2(FIELD_CENTER_Y - y, FIELD_CENTER_X - x);
    }

    public static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
