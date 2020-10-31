package org.firstinspires.ftc.teamcode.vision.operators;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class HSVRangeFilter implements MatOperator {
    private Scalar lowerThreshold = new Scalar(10, 100, 120);
    private Scalar upperThershold = new Scalar(20, 255, 255);
    private Size blur = new Size(5,5);

    public HSVRangeFilter() {

    }
    public HSVRangeFilter(Scalar lowerThreshold, Scalar upperThreshold) {
        this.lowerThreshold = lowerThreshold;
        this.upperThershold = upperThreshold;
    }
    public HSVRangeFilter(Scalar lowerThreshold, Scalar upperThreshold, Size blur) {
        this.lowerThreshold = lowerThreshold;
        this.upperThershold = upperThreshold;
        this.blur = blur;
    }

    @Override
    public Mat process(Mat img) {
        Mat mask = new Mat();
        Imgproc.cvtColor(img, mask, Imgproc.COLOR_BGR2HSV);

        Imgproc.GaussianBlur(mask, mask, blur, 1);

        Core.inRange(mask, lowerThreshold, upperThershold, mask);
        return mask;
    }
}
