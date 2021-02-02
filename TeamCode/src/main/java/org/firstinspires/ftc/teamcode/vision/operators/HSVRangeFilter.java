package org.firstinspires.ftc.teamcode.vision.operators;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.MatOperator;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Config
public class HSVRangeFilter implements MatOperator {
    public static Scalar lowerThreshold = new Scalar(10, 100, 120);
    public static Scalar upperThreshold = new Scalar(20, 255, 255);
    public static Size blur = new Size(5,5);
    public static int blurSigma = 1;

    public HSVRangeFilter() {

    }
    public HSVRangeFilter(Scalar lowerThreshold, Scalar upperThreshold) {
        HSVRangeFilter.lowerThreshold = lowerThreshold;
        HSVRangeFilter.upperThreshold = upperThreshold;
    }
    public HSVRangeFilter(Scalar lowerThreshold, Scalar upperThreshold, Size blur) {
        HSVRangeFilter.lowerThreshold = lowerThreshold;
        HSVRangeFilter.upperThreshold = upperThreshold;
        HSVRangeFilter.blur = blur;
    }

    Mat mask = new Mat();
    @Override
    public Mat process(Mat img) {
        Imgproc.cvtColor(img, mask, Imgproc.COLOR_RGB2HSV);

        Imgproc.GaussianBlur(mask, mask, blur, blurSigma);
        Core.inRange(mask, lowerThreshold, upperThreshold, mask);
        return mask;
    }
}
