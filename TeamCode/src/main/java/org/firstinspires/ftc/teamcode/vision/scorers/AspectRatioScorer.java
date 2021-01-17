package org.firstinspires.ftc.teamcode.vision.scorers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.vision.RingData;
import org.opencv.core.Rect;

import org.firstinspires.ftc.teamcode.robotlib.vision.VisionScorer;

import static org.firstinspires.ftc.teamcode.robotlib.util.MathUtil.squareError;

@Config
public class AspectRatioScorer extends VisionScorer {
    private FtcDashboard dashboard;

    public static double optimalRatio = 5.0 / 0.75;
    public static double weight = .5;

    public AspectRatioScorer() {
        dashboard = FtcDashboard.getInstance();
    }
    public AspectRatioScorer(double optimalRatio, double weight) {
        dashboard = FtcDashboard.getInstance();
        this.optimalRatio = optimalRatio;
        this.weight = weight;
    }

    @Override
    public double score(RingData ringData) {
        Rect rect = ringData.boundingRect;
        // Get w/h or h/w, whichever is bigger just incase rotation of phone.
        double ratio = ((double) rect.width) / rect.height;
        ratio = Math.max(ratio, 1.0/ratio);
        dashboard.getTelemetry().addLine("aspect ratio = " + ratio);
        return squareError(ratio, optimalRatio) * weight;
    }
}