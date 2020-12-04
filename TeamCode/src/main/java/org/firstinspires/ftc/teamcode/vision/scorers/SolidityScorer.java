package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

import robotlib.vision.VisionScorer;

import static robotlib.util.MathUtil.squareError;

@Config
public class SolidityScorer extends VisionScorer {
    private FtcDashboard dashboard;

    public static double optimalRatio = .8;
    public static double weight = 1;

    public SolidityScorer() {
        dashboard = FtcDashboard.getInstance();
    }
    public SolidityScorer(double optimalRatio, double weight) {
        dashboard = FtcDashboard.getInstance();
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
        dashboard.getTelemetry().addLine("solidity ratio = " + ratio);
        return squareError(ratio, optimalRatio) * weight;
    }
}