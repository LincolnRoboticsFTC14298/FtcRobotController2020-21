package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

@Config
public class SolidityScorer extends VisionScorer {
    public static double optimalRatio = .8;
    public static double weight = 1;

    public SolidityScorer() {

    }
    public SolidityScorer(double optimalRatio, double weight) {
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
        double ratio = Imgproc.contourArea(contour) / rect.area();
        return squareError(ratio, optimalRatio) * weight;
    }
}
