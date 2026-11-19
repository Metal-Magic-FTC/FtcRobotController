//package org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher;
//
//import static org.firstinspires.ftc.teamcode.biobuzz.comp1.launcher.LauncherConstants.*;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
///**
// * The single flywheel motor. Speed is set as BALL exit speed in m/s; the conversion to motor rad/s
// * (DcMotorEx.setVelocity(rate, AngleUnit.RADIANS)) happens here, so no ticks anywhere.
// *
// * setVelocity with an AngleUnit converts using the motor type picked in the Robot Configuration, so that motor type
// * MUST be right (goBILDA 5202/5203 bare = 28 ticks/rev) or every speed is off by the same factor.
// */
//public class Launcher {
//
//    private final DcMotorEx motor;
//    private double targetBallMps = 0;
//    private double targetMotorRad = 0;
//    private boolean saturated = false;
//
//    public Launcher(HardwareMap hardwareMap) {
//        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
//        motor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // let the flywheel coast down
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setVelocityPIDFCoefficients(VEL_P, VEL_I, VEL_D, VEL_F);
//    }
//
//    // ======================= CONVERSIONS =======================
//    private static double wheelRadiusM() {
//        return WHEEL_DIAMETER_IN / 2.0 * ShooterCalculator.IN_TO_M;
//    }
//
//    /** Ball leaves at this fraction of the wheel surface speed before losses. */
//    private static double exitFraction() {
//        return DUAL_WHEEL ? 1.0 : 0.5;
//    }
//
//    public static double ballMpsToMotorRad(double ballMps) {
//        double wheelSurface = ballMps / (ETA * exitFraction());
//        return wheelSurface / wheelRadiusM() * MOTOR_REVS_PER_WHEEL_REV;
//    }
//
//    public static double motorRadToBallMps(double motorRad) {
//        return motorRad / MOTOR_REVS_PER_WHEEL_REV * wheelRadiusM() * ETA * exitFraction();
//    }
//
//    public static double radToRpm(double rad) {
//        return rad * 60.0 / (2.0 * Math.PI);
//    }
//
//    // ======================= COMMANDS =======================
//    /** Spin up so the ball leaves at ballMps (already drag-scaled). 0 = stop. */
//    public void setBallSpeed(double ballMps) {
//        targetBallMps = Math.max(0, ballMps);
//        double rad = ballMpsToMotorRad(targetBallMps);
//        saturated = rad > MAX_MOTOR_RAD_PER_SEC;
//        targetMotorRad = Math.min(rad, MAX_MOTOR_RAD_PER_SEC);
//        motor.setVelocity(targetMotorRad, AngleUnit.RADIANS);
//    }
//
//    public void stop() {
//        targetBallMps = targetMotorRad = 0;
//        saturated = false;
//        motor.setPower(0);
//    }
//
//    // ======================= STATE =======================
//    public double motorRad() {
//        return motor.getVelocity(AngleUnit.RADIANS);
//    }
//
//    /** Estimated ball exit speed from the measured wheel speed, m/s. */
//    public double measuredBallMps() {
//        return motorRadToBallMps(motorRad());
//    }
//
//    public boolean ready() {
//        return targetMotorRad > 0 && Math.abs(motorRad() - targetMotorRad) <= VELOCITY_TOLERANCE * targetMotorRad;
//    }
//
//    /** The wanted speed needs more than MAX_MOTOR_RAD_PER_SEC (gear up, bigger wheel, or steeper hood). */
//    public boolean saturated() {
//        return saturated;
//    }
//
//    public double targetBallMps() {
//        return targetBallMps;
//    }
//
//    public double targetMotorRad() {
//        return targetMotorRad;
//    }
//}
