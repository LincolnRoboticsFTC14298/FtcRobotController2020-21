package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.vision.RingData;

import org.firstinspires.ftc.teamcode.robotlib.vision.VisionScorer;

@Config
public class AreaScorer extends VisionScorer {
    private FtcDashboard dashboard;
    public static double weight = .4;

    public AreaScorer() {
        dashboard = FtcDashboard.getInstance();
    }
    public AreaScorer(double weight) {
        dashboard = FtcDashboard.getInstance();
        this.weight = weight;
    }

    @Override
    public double score(RingData ringData) {
        double area = ringData.contourArea;
        dashboard.getTelemetry().addLine("area = " + area);
        return -area / 1000 * weight;
    }
}