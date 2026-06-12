package org.firstinspires.ftc.teamcode.summer26.aarushTests.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Final Auto Color Tracking")
public class RecogniseColorsCV extends OpMode {

    private Limelight3A limelight;
    private ColorVisionProcessor colorVision;

    @Override
    public void init() {
        // initialize
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        colorVision = new ColorVisionProcessor(limelight);

        telemetry.addData("Status", "NihalIsCakedUp");
        telemetry.update();
    }

    @Override
    public void loop() {
        colorVision.autoSelectTarget();
        if (colorVision.hasTarget()) {
            if (colorVision.getDetectedColor() == ColorVisionProcessor.ColorType.GREEN && colorVision.centered(1.5)) {
                //to be updated
            }
        }

        // 3. Driver Telemetry Output
        telemetry.addData("STATUS", "");
        telemetry.addData("best target found", colorVision.getDetectedColor());
        telemetry.addData("Target TX - Horizontal", colorVision.getTx());
        telemetry.addData("Target TY - Vertical", colorVision.getTy());
        telemetry.addData("target area", colorVision.getTa());
        telemetry.addData("estimated distance (in)", colorVision.getDistanceInches());
        telemetry.addData("centered?", colorVision.centered(1.5));

        telemetry.update();

        //driving code stuff here
    }

    public static class ColorVisionProcessor {

        private final Limelight3A limelight;

        public enum ColorType {
            PURPLE,
            GREEN,
            WHITE,
            NONE
        }

        private ColorType detectedColor = ColorType.NONE;
        private double tx = 0;
        private double ty = 0;
        private double ta = 0;

        public ColorVisionProcessor(Limelight3A limelight) {
            this.limelight = limelight;
        }

        public void autoSelectTarget() {
            double maxArea = 0;
            ColorType bestColor = ColorType.NONE;
            double bestTx = 0;
            double bestTy = 0;
            double bestTa = 0;

            // config 0 = purple, 1 = green, 5 = white
            int[] pipelinesToCheck = {0, 1, 5};
            ColorType[] correspondingColors = {ColorType.PURPLE, ColorType.GREEN, ColorType.WHITE};

            for (int i = 0; i < pipelinesToCheck.length; i++) {
                limelight.pipelineSwitch(pipelinesToCheck[i]);

                // should i use a delay? bc for hardware
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    double currentArea = result.getTa();
                    if (currentArea > maxArea) {
                        maxArea = currentArea;
                        bestColor = correspondingColors[i];
                        bestTx = result.getTx();
                        bestTy = result.getTy();
                        bestTa = currentArea;
                    }
                }
            }

            // final selected targets
            this.detectedColor = bestColor;
            this.tx = bestTx;
            this.ty = bestTy;
            this.ta = bestTa;
        }

        public boolean hasTarget() {
            return detectedColor != ColorType.NONE;
        }

        //do we need accessor and modifiers?
        public ColorType getDetectedColor() {
            return detectedColor;
        }

        public double getTx() {
            return tx;
        }

        public double getTy() {
            return ty;
        }

        public double getTa() {
            return ta;
        }

        public double getDistanceInches() {
            double limelightMountAngleDegrees = 0; // make custom
            double limelightLensHeightInches = 4;
            double goalHeightInches = 1.17;

            double angleToGoalDegrees = limelightMountAngleDegrees + Math.abs(ty);
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

            return Math.abs(goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        }

        public boolean centered(double toleranceDegrees) {
            return Math.abs(tx) < toleranceDegrees;
        }
    }
}