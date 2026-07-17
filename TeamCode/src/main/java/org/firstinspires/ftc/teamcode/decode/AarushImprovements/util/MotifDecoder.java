package org.firstinspires.ftc.teamcode.decode.AarushImprovements.util;

import org.firstinspires.ftc.teamcode.decode.AarushImprovements.subsystems.Spindexer;

/**
 * Maps AprilTag IDs from the obelisk to game-side motifs, and resolves the
 * best spindexer starting index for a given motif.
 *
 * <p>Obelisk tag IDs (DECODE game manual): 21, 22, 23.</p>
 */
public final class MotifDecoder {

    public enum Motif {
        PGP(new Spindexer.Ball[]{Spindexer.Ball.PURPLE, Spindexer.Ball.GREEN, Spindexer.Ball.PURPLE}),
        PPG(new Spindexer.Ball[]{Spindexer.Ball.PURPLE, Spindexer.Ball.PURPLE, Spindexer.Ball.GREEN}),
        GPP(new Spindexer.Ball[]{Spindexer.Ball.GREEN, Spindexer.Ball.PURPLE, Spindexer.Ball.PURPLE});

        public final Spindexer.Ball[] pattern;

        Motif(Spindexer.Ball[] pattern) {
            this.pattern = pattern;
        }

        public Spindexer.Ball getSlot(int index) {
            return pattern[((index % 3) + 3) % 3];
        }

        /** Position of the single green ball in the pattern (always 0, 1, or 2). */
        public int greenPosition() {
            for (int i = 0; i < 3; i++) {
                if (pattern[i] == Spindexer.Ball.GREEN) return i;
            }
            return -1;
        }
    }

    /** The three obelisk AprilTag IDs used in DECODE. */
    private static final int[] OBELISK_TAG_IDS = {21, 22, 23};
    /** Default motif when the tag cannot be read. */
    private static final int DEFAULT_TAG_ID = 22;

    private MotifDecoder() {
        // utility
    }

    /**
     * Map a single AprilTag ID to a motif.
     *
     * @param tagId raw tag ID (any int; out-of-range falls back to PPG)
     * @return the corresponding motif (never null)
     */
    public static Motif fromAprilTagId(int tagId) {
        switch (tagId) {
            case 21: return Motif.PGP;
            case 22: return Motif.PPG;
            case 23: return Motif.GPP;
            default: return Motif.PPG;
        }
    }

    public static int getTagId(Motif motif) {
        if (motif == null) return DEFAULT_TAG_ID;
        switch (motif) {
            case PGP: return 21;
            case PPG: return 22;
            case GPP: return 23;
            default: return DEFAULT_TAG_ID;
        }
    }

    public static boolean isValidTagId(int tagId) {
        for (int id : OBELISK_TAG_IDS) {
            if (id == tagId) return true;
        }
        return false;
    }

    /**
     * Pick the first valid tag from a list and convert to a motif.
     *
     * @param tagIds list of tag IDs, in priority order
     * @return the first matching motif, or the default motif (PPG) if none match
     */
    public static Motif fromTagIds(int... tagIds) {
        if (tagIds == null) return Motif.PPG;
        for (int id : tagIds) {
            if (isValidTagId(id)) return fromAprilTagId(id);
        }
        return Motif.PPG;
    }

    /**
     * Compute the spindexer starting index that aligns the loaded slots with
     * the given motif. The slots are treated as a circular buffer of length 3
     * and one starting offset is chosen per motif by matching the green ball.
     *
     * @param slots the 3 spindexer slots (only the position of the green ball
     *              matters; empties and purples are ignored)
     * @param motif the target motif
     * @return the index to rotate the spindexer to (0..2), or -1 if no green
     *         ball is loaded (i.e. we can't align with the motif)
     */
    public static int findBestStartIndex(Spindexer.Ball[] slots, Motif motif) {
        if (slots == null || slots.length < 3 || motif == null) return 0;

        int greenIndex = -1;
        for (int i = 0; i < 3; i++) {
            if (slots[i] == Spindexer.Ball.GREEN) {
                greenIndex = i;
                break;
            }
        }
        if (greenIndex == -1) return -1;

        int motifGreen = motif.greenPosition();
        if (motifGreen < 0) return -1;

        // Align spindexer-green with motif-green via circular offset.
        return ((greenIndex - motifGreen) % 3 + 3) % 3;
    }
}
