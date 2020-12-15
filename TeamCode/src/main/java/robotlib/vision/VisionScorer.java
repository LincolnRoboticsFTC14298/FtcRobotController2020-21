package robotlib.vision;

import org.opencv.core.MatOfPoint;

public abstract class VisionScorer {
    public abstract void updateVals(double optimalVal, double weight);
    public abstract double score(MatOfPoint contour);
}