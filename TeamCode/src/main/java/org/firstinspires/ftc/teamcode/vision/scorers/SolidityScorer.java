package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.VisionScorer;
import org.firstinspires.ftc.teamcode.vision.RingData;

import static org.firstinspires.ftc.robotlib.util.MathUtil.squareError;

@Config
public class SolidityScorer implements VisionScorer {
    private final FtcDashboard dashboard;

    public static double optimalRatio = .8;
    public static double weight = 1;

    public SolidityScorer() {
        dashboard = FtcDashboard.getInstance();
    }
    public SolidityScorer(double optimalRatio, double weight) {
        dashboard = FtcDashboard.getInstance();
        this.optimalRatio = optimalRatio;
        this.weight = weight;
    }

    @Override
    public double score(RingData ringData) {
        double ratio = ringData.getContourArea() / ringData.getConvexHullArea();
        dashboard.getTelemetry().addLine("solidity ratio = " + ratio);
        return squareError(ratio, optimalRatio) * weight;
    }

    @Override
    public double getWeight() {
        return weight;
    }
}