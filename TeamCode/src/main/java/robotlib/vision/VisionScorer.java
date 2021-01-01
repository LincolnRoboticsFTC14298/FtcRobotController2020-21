package robotlib.vision;

import org.firstinspires.ftc.teamcode.vision.RingData;

public abstract class VisionScorer {
    public abstract void updateVals(double optimalVal, double weight);
    public abstract double score(RingData ringData);
}
