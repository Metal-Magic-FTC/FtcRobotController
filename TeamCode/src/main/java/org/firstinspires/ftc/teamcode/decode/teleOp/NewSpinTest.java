package org.firstinspires.ftc.teamcode.decode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.decode.teleOp.actual.TeleV4;

import java.lang.reflect.Array;

@TeleOp(name = "!!!SpindexerTest")
public class NewSpinTest extends LinearOpMode {

    private DcMotor spinMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor");

        // Encoder setup
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set direction (flip if needed)
        spinMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Current Position", spinMotor.getCurrentPosition());
            telemetry.update();
        }
    }
//    private BallColor detectColor(NormalizedColorSensor sensor) {
//        NormalizedRGBA c = sensor.getNormalizedColors();
//        float r = c.red, g = c.green, b = c.blue;
//        float tol = 0.20f;
//
//        if (b > r*(1+tol) && b > g*(1+tol)) return BallColor.PURPLE;
//        if (g > r*(1+tol) && g > b*(1+tol)) return BallColor.GREEN;
//
//        if (r > 0.01 || g > 0.01 || b > 0.01) return BallColor.EMPTY;
//        return BallColor.EMPTY;
//    }
}