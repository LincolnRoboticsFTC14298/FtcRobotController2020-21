package org.firstinspires.ftc.teamcode.vision.scorers;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.AbstractVisionScorer;
import org.firstinspires.ftc.teamcode.vision.RingData;

@Config
public class AreaScorer extends AbstractVisionScorer {
    private final FtcDashboard dashboard;
    public static double weight = 10;
    private double area;

    public AreaScorer() {
        super("Area Scorer");
        dashboard = FtcDashboard.getInstance();
    }
    public AreaScorer(double weight) {
        super("Area Scorer");
        dashboard = FtcDashboard.getInstance();
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