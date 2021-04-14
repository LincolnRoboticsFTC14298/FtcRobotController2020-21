package org.firstinspires.ftc.teamcode.vision.scorers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.AbstractVisionScorer;
import org.firstinspires.ftc.teamcode.vision.RingData;

@Config
public class AreaScorer extends AbstractVisionScorer {
    public static double weight = 30;
    private double area;

    public AreaScorer() {
        super("Area Scorer");
    }
    public AreaScorer(double weight) {
        super("Area Scorer");
        this.weight = weight;
    }

    @Override
    public double score(RingData ringData) {
        area = ringData.getNormalizedContourArea();
        return -area * weight;
    }

    @Override
    public double getWeight() {
        return weight;
    }

    @Override
    public void updateTelemetry() {
        telemetry.put("Area", area);
    }

    @Override
    public void updateLogging() {
        Log.i("Area", String.valueOf(area));
    }
}