package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.vision.RingData;

import robotlib.vision.VisionScorer;

import static robotlib.util.MathUtil.squareError;

@Config
public class ExtentScorer extends VisionScorer {
    private FtcDashboard dashboard;

    public static double optimalRatio = .8;
    public static double weight = .7;

    public ExtentScorer() {
        dashboard = FtcDashboard.getInstance();
    }
    public ExtentScorer(double optimalRatio, double weight) {
        dashboard = FtcDashboard.getInstance();
        updateVals(optimalRatio, weight);
    }

    @Override
    public void updateVals(double optimalRatio, double weight) {
        this.optimalRatio = optimalRatio;
        this.weight = weight;
    }

    @Override
    public double score(RingData ringData) {
        double ratio = ringData.contourArea / ringData.boxArea;
        dashboard.getTelemetry().addLine("extent ratio = " + ratio);
        return squareError(ratio, optimalRatio) * weight;
    }
}