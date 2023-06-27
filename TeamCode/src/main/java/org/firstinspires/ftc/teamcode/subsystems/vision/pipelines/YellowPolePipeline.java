package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class YellowPolePipeline extends OpenCvPipeline {

    public YellowPolePipeline(double slopeVar){
        this.slopeVar = slopeVar;
    }
    private double slopeVar;

    // Set the values for the camera matrix
    private final double fx = 1013.8;
    private final double cx = 956.5;

    // Define range for bright yellow color
    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    // Convolution kernel parameters
    private final int kernelWidth = 16;

    // Calculate the distance to the pole
    @Override
    public Mat processFrame(Mat input) {
        // Convert image to HSV color space
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        // Create a mask to isolate the yellow pole
        Mat mask = new Mat();
        Core.inRange(hsv, lowerYellow, upperYellow, mask);

        // Collapse the image into one dimension by summing along the vertical axis
        Mat resultSum = new Mat();
        Core.reduce(mask, resultSum, 0, Core.REDUCE_SUM, CvType.CV_32S);

        // Create convolution kernel
        Mat kernel = Mat.ones(1, kernelWidth, CvType.CV_32F);

        // Perform convolution on the result sum
        Mat convResult = new Mat();
        Imgproc.filter2D(resultSum, convResult, -1, kernel);

        // Find the x value of the maximum in the convolution result
        double xLeftConv = Core.minMaxLoc(convResult).maxLoc.x;

        // Add the kernel width to the x value to get the x value of the maximum in the original array
        double xLeft = xLeftConv + kernelWidth / 2;

        // Calculate the slope
        double slopeVar = fx / (xLeft - cx);

        // Return the input image for display (optional)
        return input;
    }
}