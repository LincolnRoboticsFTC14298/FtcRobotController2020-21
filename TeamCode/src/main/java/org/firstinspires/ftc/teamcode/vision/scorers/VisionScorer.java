package org.firstinspires.ftc.teamcode.vision.scorers;

import org.opencv.core.MatOfPoint;

public abstract class VisionScorer {
    public abstract void updateVals(double optimalVal, double weight);
    public abstract double score(MatOfPoint contour);
    static double squareError(double val, double optimalVal) {
        double diff = optimalVal - val;
        return diff * diff;
    }
}
