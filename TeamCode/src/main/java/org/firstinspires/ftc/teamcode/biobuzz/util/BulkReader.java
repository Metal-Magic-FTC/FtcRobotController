package org.firstinspires.ftc.teamcode.biobuzz.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * REV hub bulk reading for any OpMode. Without it every encoder / velocity / digital / analog read is its own
 * ~2-3 ms trip to the hub; with it ONE trip per hub returns all of them.
 *
 *   BulkReader bulk = new BulkReader(hardwareMap);   // in init
 *   while (opModeIsActive()) {
 *       bulk.update();                               // FIRST thing, once per loop
 *       ...
 *   }
 *
 * Between update() calls every read returns the same snapshot, so never wait for an encoder to change inside one
 * loop. I2C devices (Pinpoint, IMU), the Limelight, and motor/servo writes are not affected.
 */
public class BulkReader {

    private final List<LynxModule> hubs;

    public BulkReader(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    /** Throws away the last snapshot so the next read fetches fresh data. Call once at the top of every loop. */
    public void update() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }
}
