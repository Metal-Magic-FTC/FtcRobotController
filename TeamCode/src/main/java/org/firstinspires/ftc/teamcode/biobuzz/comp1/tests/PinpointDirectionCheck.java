package org.firstinspires.ftc.teamcode.biobuzz.comp1.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Works out the Pinpoint settings from two pushes, using the RAW encoder counts (no directions, offsets or pod
 * type involved), then prints exactly what to put in Constants.localizerConstants.
 *
 *   1. INIT with the robot still.
 *   2. Push the robot straight FORWARD exactly one tile (PUSH_IN inches), no turning. Press A.
 *   3. Push the robot straight to its LEFT exactly one tile, no turning. Press B.
 *   4. Read the verdict. X starts over.
 */
@TeleOp(name = "Pinpoint Direction Check", group = "BioBuzz Tests")
public class PinpointDirectionCheck extends OpMode {

    /** How far you push in each step, inches (one field tile). */
    private static final double PUSH_IN = 24.0;

    private static final double TICKS_PER_MM_4_BAR = 19.89436789;
    private static final double TICKS_PER_MM_SWINGARM = 13.26291192;

    private GoBildaPinpointDriver odo;
    private int startX, startY;
    private int forwardDx, forwardDy, leftDx, leftDy;
    private int step = 0; // 0 = waiting for forward push, 1 = waiting for left push, 2 = done

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        telemetry.addLine("Keep the robot still, then press START");
    }

    @Override
    public void start() {
        mark();
    }

    @Override
    public void loop() {
        odo.update();
        int dx = odo.getEncoderX() - startX;
        int dy = odo.getEncoderY() - startY;

        if (gamepad1.xWasPressed()) {
            step = 0;
            mark();
        } else if (gamepad1.aWasPressed() && step == 0) {
            forwardDx = dx;
            forwardDy = dy;
            step = 1;
            mark();
        } else if (gamepad1.bWasPressed() && step == 1) {
            leftDx = dx;
            leftDy = dy;
            step = 2;
        }

        if (step == 0) {
            telemetry.addLine("STEP 1: push straight FORWARD " + PUSH_IN + " in, then press A");
        } else if (step == 1) {
            telemetry.addLine("STEP 2: push straight LEFT " + PUSH_IN + " in, then press B");
        }
        if (step < 2) {
            telemetry.addData("X port ticks since last step", dx);
            telemetry.addData("Y port ticks since last step", dy);
            return;
        }

        boolean forwardOnX = Math.abs(forwardDx) > Math.abs(forwardDy);
        boolean leftOnY = Math.abs(leftDy) > Math.abs(leftDx);
        telemetry.addData("Forward push", "X port %d, Y port %d", forwardDx, forwardDy);
        telemetry.addData("Left push", "X port %d, Y port %d", leftDx, leftDy);
        telemetry.addLine("-----------------------------");

        if (!forwardOnX && !leftOnY) {
            telemetry.addLine("PODS ARE IN THE WRONG PORTS: swap the two pod cables on the Pinpoint, then run again.");
        } else if (!forwardOnX || !leftOnY) {
            telemetry.addLine("Unclear: one pod moved in both pushes. Push straighter and press X to redo.");
        } else {
            telemetry.addLine("Ports are correct. Put this in Constants:");
            telemetry.addData("forwardEncoderDirection", forwardDx > 0 ? "FORWARD" : "REVERSED");
            telemetry.addData("strafeEncoderDirection", leftDy > 0 ? "FORWARD" : "REVERSED");
            telemetry.addData("Forward pod", podType(forwardDx));
            telemetry.addData("Strafe pod", podType(leftDy));
        }
        telemetry.addLine("X = start over");
    }

    private void mark() {
        odo.update();
        startX = odo.getEncoderX();
        startY = odo.getEncoderY();
    }

    private static String podType(int ticks) {
        double ticksPerMm = Math.abs(ticks) / (PUSH_IN * 25.4);
        String guess = Math.abs(ticksPerMm - TICKS_PER_MM_4_BAR) < Math.abs(ticksPerMm - TICKS_PER_MM_SWINGARM)
                ? "goBILDA_4_BAR_POD" : "goBILDA_SWINGARM_POD";
        return String.format("%s  (measured %.2f ticks/mm; 4-bar 19.89, swingarm 13.26)", guess, ticksPerMm);
    }
}
