package org.firstinspires.ftc.teamcode.vision.scorers;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class AreaScorer extends VisionScorer {
    private double realWidth = 5.0; // In inches
    private double optimalArea = 7800; // In inches ^ 2
    private double weight = .0000001;

    public AreaScorer() {

    }
    public AreaScorer(double optimalArea, double weight) {
        updateVals(optimalArea, weight);
    }

    @Override
    public void updateVals(double optimalArea, double weight) {
        this.optimalArea = optimalArea;
        this.weight = weight;
    }

    @Override
    public double score(MatOfPoint contour) {
        Rect rect = Imgproc.boundingRect(contour);
        // Get h/w or w/h, whichever is smaller just encase rotation of phone.

        double area = Imgproc.contourArea(contour);
        return squareError(area, optimalArea) * weight;
    }
}

