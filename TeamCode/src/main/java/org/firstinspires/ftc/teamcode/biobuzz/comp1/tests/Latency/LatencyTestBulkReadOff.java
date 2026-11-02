package org.firstinspires.ftc.teamcode.biobuzz.comp1.tests.Latency;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** LatencyTestTeleOp WITHOUT bulk reading (every read goes to the hub). Compare against LatencyTestBulkReadOn. */
@TeleOp(name = "Latency Test: Bulk Read OFF", group = "BioBuzz Tests")
public class LatencyTestBulkReadOff extends LatencyTestTeleOp {

    @Override
    protected boolean useBulkReading() {
        return false;
    }
}
