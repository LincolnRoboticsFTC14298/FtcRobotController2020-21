package org.firstinspires.ftc.teamcode.vision.scorers;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;

public interface VisionScorer {
    double score(Rect rect, MatOfPoint contour);
}
