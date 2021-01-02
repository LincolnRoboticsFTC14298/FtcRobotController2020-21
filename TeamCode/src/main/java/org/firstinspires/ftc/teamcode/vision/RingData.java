package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import static org.firstinspires.ftc.teamcode.vision.VisionUtil.pickPoints;
import static org.firstinspires.ftc.teamcode.vision.VisionUtil.rectCenter;

public class RingData {
    public MatOfPoint contour;
    public double contourArea;
    public Point centroid;

    public double convexHullArea;

    public Rect boundingRect;
    public double boxArea;
    public Point boxCenter;
    public Size boxSize;

    public RingData(MatOfPoint contour) {
        this.contour = contour;
        contourArea = Imgproc.contourArea(contour);

        Moments p = Imgproc.moments(contour);
        centroid = new Point(p.get_m10() / p.get_m00(),
                             p.get_m01() / p.get_m00());

        MatOfInt hullIndices = new MatOfInt();
        Imgproc.convexHull(contour, hullIndices);
        MatOfPoint2f hull = pickPoints(contour, hullIndices);
        convexHullArea = Imgproc.contourArea(hull);

        boundingRect = Imgproc.boundingRect(contour);
        boxArea = boundingRect.area();
        boxCenter = rectCenter(boundingRect);
        boxSize = boundingRect.size();
    }
}
