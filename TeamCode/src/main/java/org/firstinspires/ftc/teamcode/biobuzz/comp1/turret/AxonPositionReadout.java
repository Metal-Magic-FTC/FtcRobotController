package org.firstinspires.ftc.teamcode.biobuzz.comp1.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

/**
 * Read-only: shows the Axon MAX MK2's position from its analog feedback wire. The servo is kept limp (PWM off),
 * so turn it by hand.
 *
 * Config: servo port "turretServo", feedback wire on an analog port as an "Analog Input" named "turretEncoder".
 * gamepad1 A - zero the continuous angle here.
 */
@TeleOp(name = "Axon Position Readout", group = "BioBuzz Tests")
public class AxonPositionReadout extends OpMode {

    private AnalogInput feedback;
    private double lastDeg;
    private double totalDeg = 0; // keeps counting past 360
    private double zeroDeg = 0;

    @Override
    public void init() {
        hardwareMap.get(CRServoImplEx.class, AxonTurret.SERVO_NAME).setPwmDisable();
        feedback = hardwareMap.get(AnalogInput.class, AxonTurret.ENCODER_NAME);
        lastDeg = readDeg();
    }

    @Override
    public void init_loop() {
        loop();
    }

    @Override
    public void loop() {
        double deg = readDeg();
        totalDeg += TurretConstants.wrap180(deg - lastDeg);
        lastDeg = deg;
        if (gamepad1.aWasPressed()) zeroDeg = totalDeg;

        telemetry.addData("Voltage", "%.3f V", feedback.getVoltage());
        telemetry.addData("Angle (0..360)", "%.1f°", deg);
        telemetry.addData("Since zero", "%.1f°", totalDeg - zeroDeg);
        telemetry.addLine("A = zero here");
    }

    /** Horn angle 0..360 from the feedback voltage. */
    private double readDeg() {
        return feedback.getVoltage() / feedback.getMaxVoltage() * 360.0;
    }
}
