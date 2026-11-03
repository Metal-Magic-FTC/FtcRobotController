package org.firstinspires.ftc.teamcode.biobuzz.comp1.tests.Latency;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** LatencyTestTeleOp WITH bulk reading. Compare against LatencyTestBulkReadOff. */
@TeleOp(name = "Latency Test: Bulk Read ON", group = "BioBuzz Tests")
public class LatencyTestBulkReadOn extends LatencyTestTeleOp {

    @Override
    protected boolean useBulkReading() {
        return true;
    }
}
