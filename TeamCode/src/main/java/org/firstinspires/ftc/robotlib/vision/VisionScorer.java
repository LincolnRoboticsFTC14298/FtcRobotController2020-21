package org.firstinspires.ftc.robotlib.vision;

import org.firstinspires.ftc.teamcode.vision.RingData;

public interface VisionScorer {
    double score(RingData ringData);
    double getWeight();
}
