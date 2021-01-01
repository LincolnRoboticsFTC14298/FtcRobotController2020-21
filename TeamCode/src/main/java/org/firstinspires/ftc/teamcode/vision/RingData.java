package org.firstinspires.ftc.teamcode.vision;

import org.jetbrains.annotations.NotNull;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import static org.firstinspires.ftc.teamcode.vision.VisionUtil.pickPoints;
import static org.firstinspires.ftc.teamcode.vision.VisionUtil.rectCenter;

public class RingData implements Comparable<RingData> {
    public MatOfPoint contour;
    public double contourArea;
    public Point centroidPoint;
    public double[] centroid;

    public double convexHullArea;

    public Rect boundingRect;
    public double boxArea;
    public Point boxCenterPoint;
    public double[] boxCenter;
    public int[] boxDimensions;

    public RingData(MatOfPoint contour) {
        this.contour = contour;
        contourArea = Imgproc.contourArea(contour);

        Moments p = Imgproc.moments(contour);
        centroidPoint = new Point(p.get_m10() / p.get_m00(),
                                  p.get_m01() / p.get_m00());
        centroid = new double[]{centroidPoint.x, centroidPoint.y};

        MatOfInt hullIndices = new MatOfInt();
        Imgproc.convexHull(contour, hullIndices);
        MatOfPoint2f hull = pickPoints(contour, hullIndices);
        convexHullArea = Imgproc.contourArea(hull);

        boundingRect = Imgproc.boundingRect(contour);
        boxArea = boundingRect.area();
        boxCenterPoint = rectCenter(boundingRect);
        boxCenter = new double[]{boxCenterPoint.x, boxCenterPoint.y};
        boxDimensions = new int[]{boundingRect.width, boundingRect.height};
    }

    @Override
    public int compareTo(@NotNull RingData r1) {
        return (int) (this.contourArea - r1.contourArea);
    }
}
