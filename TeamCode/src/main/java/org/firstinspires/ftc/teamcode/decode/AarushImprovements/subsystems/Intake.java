package org.firstinspires.ftc.teamcode.decode.AarushImprovements.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake: single motor that pulls balls into the spindexer.
 *
 * <p>Default hardware name: {@code intakeMotor}. Sign convention: negative
 * power = intaking, positive = ejecting.</p>
 */
public class Intake {

    private final DcMotor intakeMotor;

    private static final double INTAKE_POWER_DEFAULT = -0.8;
    private static final double INTAKE_POWER_SLOW = -0.6;
    private static final double OUTTAKE_POWER_DEFAULT = 0.6;
    private static final double OUTTAKE_POWER_FAST = 0.8;

    private boolean isRunning = false;
    private boolean isIntaking = false;

    public Intake(HardwareMap hardwareMap, String intakeMotorName) {
        intakeMotor = hardwareMap.get(DcMotor.class, intakeMotorName);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, "intakeMotor");
    }

    public void intake() {
        intakeMotor.setPower(INTAKE_POWER_DEFAULT);
        isRunning = true;
        isIntaking = true;
    }

    public void intake(double power) {
        double actualPower = power > 0 ? -power : power;
        intakeMotor.setPower(actualPower);
        isRunning = true;
        isIntaking = true;
    }

    public void intakeSlow() {
        intakeMotor.setPower(INTAKE_POWER_SLOW);
        isRunning = true;
        isIntaking = true;
    }

    public void outtake() {
        intakeMotor.setPower(OUTTAKE_POWER_DEFAULT);
        isRunning = true;
        isIntaking = false;
    }

    public void outtake(double power) {
        double actualPower = power < 0 ? -power : power;
        intakeMotor.setPower(actualPower);
        isRunning = true;
        isIntaking = false;
    }

    public void outtakeFast() {
        intakeMotor.setPower(OUTTAKE_POWER_FAST);
        isRunning = true;
        isIntaking = false;
    }

    /**
     * Set the motor power directly. Negative = intake, positive = outtake.
     */
    public void setPower(double power) {
        intakeMotor.setPower(power);
        isRunning = power != 0;
        isIntaking = power < 0;
    }

    public void stop() {
        intakeMotor.setPower(0);
        isRunning = false;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public boolean isIntaking() {
        return isIntaking && isRunning;
    }

    public boolean isOuttaking() {
        return !isIntaking && isRunning;
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public void reverseDirection() {
        intakeMotor.setDirection(
                intakeMotor.getDirection() == DcMotorSimple.Direction.FORWARD
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        intakeMotor.setDirection(direction);
    }

    public String getTelemetryString() {
        String state = !isRunning ? "STOPPED" : (isIntaking ? "INTAKING" : "OUTTAKING");
        return String.format("Intake: %s (%.2f)", state, intakeMotor.getPower());
    }
}
