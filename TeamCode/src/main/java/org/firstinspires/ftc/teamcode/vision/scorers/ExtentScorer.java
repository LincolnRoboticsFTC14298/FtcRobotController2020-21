package org.firstinspires.ftc.teamcode.vision.scorers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.AbstractVisionScorer;
import org.firstinspires.ftc.teamcode.vision.RingData;

import static org.firstinspires.ftc.robotlib.util.MathUtil.squareError;

@Config
public class ExtentScorer extends AbstractVisionScorer {
    public static double optimalRatio = .8;
    public static double weight = .7;
    private double ratio;

    public ExtentScorer() {
        super("Extent Scorer");
    }
    public ExtentScorer(double optimalRatio, double weight) {
        super("Extent Scorer");
        this.optimalRatio = optimalRatio;
        this.weight = weight;
    }

    @Override
    public double score(RingData ringData) {
        ratio = ringData.getContourArea() / ringData.getBoxArea();
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
        Log.i("Extent", String.valueOf(ratio));
    }
}