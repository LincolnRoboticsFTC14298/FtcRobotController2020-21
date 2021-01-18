package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.VisionScorer;
import org.firstinspires.ftc.teamcode.vision.RingData;

@Config
public class AreaScorer extends VisionScorer {
    private final FtcDashboard dashboard;
    public static double weight = .4;

    public AreaScorer() {
        dashboard = FtcDashboard.getInstance();
    }
    public AreaScorer(double weight) {
        dashboard = FtcDashboard.getInstance();
        AreaScorer.weight = weight;
    }

    @Override
    public double score(RingData ringData) {
        double area = ringData.contourArea;
        dashboard.getTelemetry().addLine("area = " + area);
        return -area / 1000 * weight;
    }
}