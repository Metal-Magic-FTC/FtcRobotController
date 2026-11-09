package org.firstinspires.ftc.teamcode.biobuzz.comp1.tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * REV 2m Distance Sensor (laser time-of-flight) test. Shows the live distance and prints a line to the console
 * every time it changes by at least the change threshold:
 *   - Driver Station: the log at the bottom of telemetry (newest first)
 *   - Logcat / robot log: tag "DistanceSensorTest"
 *
 * Any DistanceSensor works (e.g. the REV Color Sensor V3's distance reading) - only the 2m sensor shows the
 * timeout flag. The sensor is I2C, so bulk reading does not speed it up; "Read time" shows the real cost.
 *
 * CONFIG: an I2C port, device type "REV 2m Distance Sensor", named SENSOR_NAME.
 *
 * gamepad1 (works in INIT too, so you can set it up before START):
 *   dpad up / down   - raise / lower the change threshold by one step
 *   right / left bumper - bigger / smaller step size
 *   A                - next unit (mm, cm, in)
 *   B                - clear the console and min/max
 */
@TeleOp(name = "BioBuzz Distance Sensor Test", group = "BioBuzz Tests")
public class DistanceSensorTest extends LinearOpMode {

    private static final String SENSOR_NAME = "distanceSensor";
    private static final String LOG_TAG = "DistanceSensorTest";
    /** Readings at or past this are "out of range" (the 2m sensor reports ~8190 mm when it sees nothing). */
    private static final double OUT_OF_RANGE_MM = 8000;
    private static final int CONSOLE_LINES = 12;

    private static final DistanceUnit[] UNITS = {DistanceUnit.MM, DistanceUnit.CM, DistanceUnit.INCH};
    /** Threshold step sizes in mm; bumpers move through these. */
    private static final double[] STEPS_MM = {1, 5, 10, 25, 50};

    private DistanceSensor sensor;
    private final ElapsedTime runtime = new ElapsedTime();

    private int unitIndex = 1;                 // cm
    private int stepIndex = 2;                 // 10 mm
    private double thresholdMm = 10;
    private double lastLoggedMm = Double.NaN;  // NaN = nothing logged yet
    private boolean lastOutOfRange = false;
    private double minMm = Double.NaN, maxMm = Double.NaN;

    @Override
    public void runOpMode() {
        sensor = hardwareMap.get(DistanceSensor.class, SENSOR_NAME);

        telemetry.log().setCapacity(CONSOLE_LINES);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        log("Started - logging changes of " + format(thresholdMm) + " or more");

        while (opModeInInit()) {
            loopOnce("INIT - set up with the gamepad, then press START");
        }
        if (isStopRequested()) return;

        log("START");
        while (opModeIsActive()) {
            loopOnce("Running");
        }
    }

    private void loopOnce(String status) {
        handleControls();

        ElapsedTime readTimer = new ElapsedTime();
        double mm = sensor.getDistance(DistanceUnit.MM);
        double readMs = readTimer.milliseconds();
        boolean outOfRange = Double.isNaN(mm) || mm >= OUT_OF_RANGE_MM;

        if (!outOfRange) {
            minMm = Double.isNaN(minMm) ? mm : Math.min(minMm, mm);
            maxMm = Double.isNaN(maxMm) ? mm : Math.max(maxMm, mm);
        }
        logIfChanged(mm, outOfRange);

        telemetry.addData("Status", status);
        telemetry.addData("Distance", outOfRange ? "OUT OF RANGE" : format(mm));
        telemetry.addData("Min / Max", "%s / %s", format(minMm), format(maxMm));
        telemetry.addData("Change threshold", "%s  (step %s)", format(thresholdMm), format(STEPS_MM[stepIndex]));
        telemetry.addData("Read time", "%.1f ms", readMs);
        if (sensor instanceof Rev2mDistanceSensor) {
            telemetry.addData("Sensor timeout", ((Rev2mDistanceSensor) sensor).didTimeoutOccur());
        }
        telemetry.addLine("dpad ↑↓ threshold | bumpers step | A unit | B clear");
        telemetry.update();
    }

    private void handleControls() {
        double oldThreshold = thresholdMm;
        if (gamepad1.dpadUpWasPressed()) thresholdMm += STEPS_MM[stepIndex];
        if (gamepad1.dpadDownWasPressed()) thresholdMm = Math.max(STEPS_MM[0], thresholdMm - STEPS_MM[stepIndex]);
        if (thresholdMm != oldThreshold) log("Threshold -> " + format(thresholdMm));

        if (gamepad1.rightBumperWasPressed()) stepIndex = Math.min(STEPS_MM.length - 1, stepIndex + 1);
        if (gamepad1.leftBumperWasPressed()) stepIndex = Math.max(0, stepIndex - 1);
        if (gamepad1.aWasPressed()) unitIndex = (unitIndex + 1) % UNITS.length;
        if (gamepad1.bWasPressed()) {
            telemetry.log().clear();
            minMm = maxMm = Double.NaN;
            lastLoggedMm = Double.NaN;
            lastOutOfRange = false;
        }
    }

    /** Prints a console line when the reading moved by >= threshold, or went in / out of range. */
    private void logIfChanged(double mm, boolean outOfRange) {
        if (outOfRange) {
            if (!lastOutOfRange) log("Out of range");
            lastOutOfRange = true;
            return;
        }
        if (lastOutOfRange || Double.isNaN(lastLoggedMm)) {
            log("In range: " + format(mm));
        } else {
            double delta = mm - lastLoggedMm;
            if (Math.abs(delta) < thresholdMm) return;
            log(String.format("%s  (%s%s)", format(mm), delta > 0 ? "+" : "-", format(Math.abs(delta))));
        }
        lastLoggedMm = mm;
        lastOutOfRange = false;
    }

    private void log(String message) {
        String line = String.format("[%6.2fs] %s", runtime.seconds(), message);
        telemetry.log().add(line);
        RobotLog.ii(LOG_TAG, line);
    }

    /** mm value in the selected unit, e.g. "12.3 cm". */
    private String format(double mm) {
        if (Double.isNaN(mm)) return "--";
        DistanceUnit unit = UNITS[unitIndex];
        double value = unit.fromMm(mm);
        String suffix = unit == DistanceUnit.INCH ? "in" : unit == DistanceUnit.CM ? "cm" : "mm";
        return String.format(unit == DistanceUnit.MM ? "%.0f %s" : "%.1f %s", value, suffix);
    }
}
