package org.firstinspires.ftc.teamcode.subsystems.vision.pipelines;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;

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

    private Mat img;

    // Calculate the distance to the pole
    @Override
    public Mat processFrame(Mat input) {
        img = input;
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

    private int captureCounter = 0;
    public String CaptureImage(){
        Mat image2Save = img;
        String fullFileName = String.format("-%d.png",captureCounter++);

        if(null != image2Save)
            if (saveOpenCvImageToFile(fullFileName, image2Save))
                return fullFileName;
        return "not saved";
    }

    public static final File VISION_FOLDER =
            new File(AppUtil.ROOT_FOLDER + "/vision/");
    private boolean saveOpenCvImageToFile(String filename, Mat mat) {

        Mat mIntermediateMat = new Mat();
        Imgproc.cvtColor(mat, mIntermediateMat, Imgproc.COLOR_BGR2RGB, 3);

        boolean mkdirs = VISION_FOLDER.mkdirs();
        File file = new File(VISION_FOLDER, filename);
        boolean savedSuccessfully = Imgcodecs.imwrite(file.toString(), mat);
        return  savedSuccessfully;
    }
}