package org.firstinspires.ftc.teamcode.metalmagic25summer.utility.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//Thresholding is checking if the pixels in the camera frame are in a certain color range.
//If not, they are colored black, and if they are, they are colored white.
@Disabled
public class BlueCV extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        //Min and Max values for the color blue
        Scalar lower_blue = new Scalar(85, 50, 40);
        Scalar upper_blue = new Scalar(135, 255, 255);

        //Creating Mat to store input and then converting it from RGB to HSV (since HSV is more consistent)
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        //Creating a mat to store the threshold and then making the threshold using min and max blue values
        Mat threshold = new Mat();
        Core.inRange(hsvMat, lower_blue, upper_blue, threshold);

        return threshold;
    }
}
