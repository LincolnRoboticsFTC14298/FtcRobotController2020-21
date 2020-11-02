package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

@Config
public class AreaScorer extends VisionScorer {
    public static double realWidth = 5.0; // In inches
    public static double optimalArea = 7800; // In inches ^ 2
    public static double weight = .0000001;

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
        double area = Imgproc.contourArea(contour);
        return squareError(area, optimalArea) * weight;
    }
}