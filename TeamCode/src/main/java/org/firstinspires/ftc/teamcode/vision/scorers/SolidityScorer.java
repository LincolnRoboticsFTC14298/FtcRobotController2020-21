package org.firstinspires.ftc.teamcode.vision.scorers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.AbstractVisionScorer;
import org.firstinspires.ftc.teamcode.vision.RingData;

import static org.firstinspires.ftc.robotlib.util.MathUtil.squareError;

@Config
public class SolidityScorer extends AbstractVisionScorer {
    public static double optimalRatio = .8;
    public static double weight = 1;
    private double ratio;

    public SolidityScorer() {
        super("Solidity Scorer");
    }
    public SolidityScorer(double optimalRatio, double weight) {
        super("Solidity Scorer");
        this.optimalRatio = optimalRatio;
        this.weight = weight;
    }

    @Override
    public double score(RingData ringData) {
        ratio = ringData.getContourArea() / ringData.getConvexHullArea();
        return squareError(ratio, optimalRatio) * weight;
    }

    @Override
    public double getWeight() {
        return weight;
    }

    @Override
    public void updateTelemetry() {
        telemetry.put("Ratio", ratio);
    }

    @Override
    public void updateLogging() {
        Log.i("Solidity", String.valueOf(ratio));
    }
}