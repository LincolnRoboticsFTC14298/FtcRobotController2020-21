package org.firstinspires.ftc.teamcode.vision.scorers;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.AbstractVisionScorer;
import org.firstinspires.ftc.teamcode.vision.RingData;
import org.opencv.core.Rect;

import static org.firstinspires.ftc.robotlib.util.MathUtil.squareError;


@Config
public class AspectRatioScorer extends AbstractVisionScorer {
    private final FtcDashboard dashboard;

    public static double optimalRatio = 5.0 / 0.75;
    public static double weight = .5;
    private double ratio;

    public AspectRatioScorer() {
        super("Aspect Ratio Scorer");
        dashboard = FtcDashboard.getInstance();
    }
    public AspectRatioScorer(double optimalRatio, double weight) {
        super("Aspect Ratio Scorer");
        dashboard = FtcDashboard.getInstance();
        this.optimalRatio = optimalRatio;
        this.weight = weight;
    }

    @Override
    public double score(RingData ringData) {
        Rect rect = ringData.getBoundingRect();
        ratio = ((double) rect.width) / rect.height;
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
        Log.i("Aspect Ratio", String.valueOf(ratio));
    }
}