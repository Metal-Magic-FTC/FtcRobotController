package org.firstinspires.ftc.teamcode.decode.util;

import org.firstinspires.ftc.teamcode.decode.subsystems.Spindexer;

public class MotifDecoder {

    public enum Motif {
        PGP(new Spindexer.Ball[]{Spindexer.Ball.PURPLE, Spindexer.Ball.GREEN, Spindexer.Ball.PURPLE}),
        PPG(new Spindexer.Ball[]{Spindexer.Ball.PURPLE, Spindexer.Ball.PURPLE, Spindexer.Ball.GREEN}),
        GPP(new Spindexer.Ball[]{Spindexer.Ball.GREEN, Spindexer.Ball.PURPLE, Spindexer.Ball.PURPLE});

        public final Spindexer.Ball[] pattern;

        Motif(Spindexer.Ball[] pattern) {
            this.pattern = pattern;
        }

        public Spindexer.Ball getSlot(int index) {
            return pattern[index % 3];
        }

        public boolean matchesSlot(int index, Spindexer.Ball ball) {
            return pattern[index % 3] == ball;
        }
    }

    private static final int[] OBELISK_TAG_IDS = {21, 22, 23};

    public static Motif fromAprilTagId(int tagId) {
        switch (tagId) {
            case 21: return Motif.PGP;
            case 22: return Motif.PPG;
            case 23: return Motif.GPP;
            default: return Motif.PPG;
        }
    }

    public static int getTagId(Motif motif) {
        switch (motif) {
            case PGP: return 21;
            case PPG: return 22;
            case GPP: return 23;
            default: return 22;
        }
    }

    public static boolean isValidTagId(int tagId) {
        for (int id : OBELISK_TAG_IDS) {
            if (id == tagId) return true;
        }
        return false;
    }

    public static Motif fromTagIds(int[] tagIds) {
        for (int id : tagIds) {
            if (isValidTagId(id)) return fromAprilTagId(id);
        }
        return Motif.PPG;
    }

    public static int findBestStartIndex(Spindexer.Ball[] slots, Motif motif) {
        if (slots.length < 3) return 0;

        for (int start = 0; start < 3; start++) {
            boolean match = true;
            for (int i = 0; i < 3; i++) {
                int slotIdx = (start + i) % 3;
                if (slots[slotIdx] != Spindexer.Ball.EMPTY && slots[slotIdx] != motif.getSlot(i)) {
                    match = false;
                    break;
                }
            }
            if (match) return start;
        }
        return 0;
    }
}
