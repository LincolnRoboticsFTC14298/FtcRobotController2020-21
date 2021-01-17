package org.firstinspires.ftc.teamcode.robotlib.vision;

import org.firstinspires.ftc.teamcode.vision.RingData;

public abstract class VisionScorer {
    public static double weight = 1;
    public abstract double score(RingData ringData);
}
