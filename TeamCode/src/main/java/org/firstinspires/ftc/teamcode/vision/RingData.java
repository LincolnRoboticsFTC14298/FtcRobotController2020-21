package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.util.VisionUtil;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import static org.firstinspires.ftc.teamcode.util.VisionUtil.normalizePoint;
import static org.firstinspires.ftc.teamcode.util.VisionUtil.pickPoints;
import static org.firstinspires.ftc.teamcode.util.VisionUtil.rectCenter;

public class RingData {
    private final MatOfPoint contour;
    private final double contourArea;
    private final Point centroid;

    private final double convexHullArea;

    private final Rect boundingRect;
    private final double boxArea;
    private final Point boxCenter;
    private final Size boxSize;

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
    public double getNormalizedContourArea() {
        return VisionUtil.normalizeArea(contourArea);
    }

    public Point getCentroid() {
        return centroid;
    }
    public Point getNormalizedCentroid() {
        return VisionUtil.normalizePoint(centroid);
    }

    public double getConvexHullArea() {
        return convexHullArea;
    }
    public double getNormalizedConvexHullArea() {
        return VisionUtil.normalizeArea(convexHullArea);
    }

    public Rect getBoundingRect() {
        return boundingRect;
    }
    public Rect getNormalizedBoundingRect() {
        return VisionUtil.normalizeRect(boundingRect);
    }

    public double getBoxArea() {
        return boxArea;
    }
    public double getNormalizedBoxArea() {
        return VisionUtil.normalizeArea(boxArea);
    }

    public Point getBoxCenter() {
        return boxCenter;
    }
    public Point getNormalizedBoxCenter() {
        return normalizePoint(boxCenter);
    }

    public Size getBoxSize() {
        return boxSize;
    }
    public Size getNormalizedBoxSize() {
        return VisionUtil.normalizeSize(boxSize);
    }
}
