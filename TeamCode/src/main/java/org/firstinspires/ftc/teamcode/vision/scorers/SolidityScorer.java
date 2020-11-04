package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.util.MathUtil.squareError;

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
        MatOfInt hull = new MatOfInt();
        Imgproc.convexHull(contour, hull);
        double ratio = Imgproc.contourArea(contour) / Imgproc.contourArea(hull);
        return squareError(ratio, optimalRatio) * weight;
    }
}