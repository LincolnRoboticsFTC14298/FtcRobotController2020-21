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
    private MatOfPoint contour;
    private double contourArea;
    private Point centroid;

    private double convexHullArea;

    private Rect boundingRect;
    private double boxArea;
    private Point boxCenter;
    private Size boxSize;

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

    public MatOfPoint getContour() {
        return contour;
    }
    public double getContourArea() {
        return contourArea;
    }
    public Point getCentroid() {
        return centroid;
    }
    public double getConvexHullArea() {
        return convexHullArea;
    }
    public Rect getBoundingRect() {
        return boundingRect;
    }
    public double getBoxArea() {
        return boxArea;
    }
    public Point getBoxCenter() {
        return boxCenter;
    }
    public Size getBoxSize() {
        return boxSize;
    }
}
