package org.firstinspires.ftc.teamcode.vision.scorers;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.util.MathUtil.squareError;

@Config
public class AspectRatioScorer extends VisionScorer {
    public static double optimalRatio = 5.0 / 0.75;
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
        // Get w/h or h/w, whichever is bigger just incase rotation of phone.
        double ratio = ((double) rect.width) / rect.height;
        ratio = Math.max(ratio, 1.0/ratio);
        return squareError(ratio, optimalRatio) * weight;
    }
}