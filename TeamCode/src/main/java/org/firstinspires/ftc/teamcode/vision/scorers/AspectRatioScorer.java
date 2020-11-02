package org.firstinspires.ftc.teamcode.vision.scorers;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

@Config
public class AspectRatioScorer extends VisionScorer {
    public static double optimalRatio = 0.75 / 5.0;
    public static double weight = 3;

    public AspectRatioScorer() {

    }
    public AspectRatioScorer(double optimalRatio, double weight) {
        updateVals(optimalRatio, weight);
    }

    @Override
    public void updateVals(double optimalRatio, double weight) {
        this.optimalRatio = optimalRatio;
        this.weight = weight;
    }

    @Override
    public double score(MatOfPoint contour) {
        Rect rect = Imgproc.boundingRect(contour);
        // Get h/w or w/h, whichever is smaller just incase rotation of phone.
        double ratio = (double) rect.height / rect.width;
        ratio = Math.min(ratio, 1.0/ratio);
        return squareError(ratio, optimalRatio) * weight;
    }
}
