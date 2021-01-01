package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.vision.RingData;

import robotlib.vision.VisionScorer;

import static robotlib.util.MathUtil.squareError;

@Config
public class AreaScorer extends VisionScorer {
    private FtcDashboard dashboard;

    public static double optimalArea = 7800; // In inches ^ 2
    public static double weight = .4;

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
    public double score(RingData ringData) {
        double area = ringData.contourArea;
        dashboard.getTelemetry().addLine("area = " + area);
        return squareError(area/1000.0, optimalArea/1000.0) * weight;
    }
}