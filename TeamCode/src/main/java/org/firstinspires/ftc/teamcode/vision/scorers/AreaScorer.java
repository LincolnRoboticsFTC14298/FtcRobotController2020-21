package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

import static org.firstinspires.ftc.teamcode.util.MathUtil.squareError;

@Config
public class AreaScorer extends VisionScorer {
    private FtcDashboard dashboard;

    public static double optimalArea = 7800; // In inches ^ 2
    public static double weight = .0000001;

    public AreaScorer() {
        dashboard = FtcDashboard.getInstance();
    }
    public AreaScorer(double optimalArea, double weight) {
        dashboard = FtcDashboard.getInstance();
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
        dashboard.getTelemetry().addLine("area = " + area);
        return squareError(area, optimalArea) * weight;
    }
}