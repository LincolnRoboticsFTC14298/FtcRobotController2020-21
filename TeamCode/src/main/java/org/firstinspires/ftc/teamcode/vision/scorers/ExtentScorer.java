package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.util.MathUtil.squareError;

@Config
public class ExtentScorer extends VisionScorer {
    public static double optimalRatio = .8;
    public static double weight = 1;

    public ExtentScorer() {

    }
    public ExtentScorer(double optimalRatio, double weight) {
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
        double ratio = Imgproc.contourArea(contour) / rect.area();
        return squareError(ratio, optimalRatio) * weight;
    }
}