package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class VisionUtil {
    public static ArrayList<MatOfPoint> findContours(Mat mask, double threshold1, double threshold2) {
        Mat cannyOutput = new Mat();
        Imgproc.Canny(mask, cannyOutput, threshold1, threshold2);
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        // Consider using chain approx none
        Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

    public static Point rectCenter(Rect rect) {
        double x = rect.x + rect.width / 2.0;
        double y = rect.y + rect.height / 2.0;
        return new Point(x, y);
    }

    public static Rect[] getBoundingRects(
            List<MatOfPoint> contours, Mat dst, boolean displayContours,
            boolean displayAllBoxes, int thickness) {
        Mat hierarchy = new Mat();
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRects = new Rect[contours.size()];
        // Find Rects
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRects[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Display contours and boxes
        if (displayContours) {
            Scalar color = generateColor();
            drawContours(dst, contoursPoly, color, thickness);
        }
        if (displayAllBoxes) {
            Scalar color = generateColor();
            drawRects(dst, boundRects, color, thickness);
        }
        return boundRects;
    }
    public static Rect[] getBoundingRects(List<MatOfPoint> contours) {
        return getBoundingRects(contours, null, false, false, 0);
    }

    public static RotatedRect[] getBoundingRotatedRects(
            List<MatOfPoint> contours, Mat dst, boolean displayContours,
            boolean displayAllBoxes, int thickness) {

        Mat hierarchy = new Mat();
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        RotatedRect[] boundRects = new RotatedRect[contours.size()];
        // Find Rects
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
        }

        // Display contours and boxes
        if (displayContours) {
            Scalar color = generateColor();
            drawContours(dst, contoursPoly, color, thickness);
        }
        if (displayAllBoxes) {
            Scalar color = generateColor();
            drawRotatedRects(dst, boundRects, color, thickness);
        }
        return boundRects;
    }
    public static RotatedRect[] getBoundingRotatedRects(List<MatOfPoint> contours) {
        return getBoundingRotatedRects(contours, null, false, false, 0);
    }

    // Todo: may remove
    // Original
    private static RotatedRect[] getBoundingRects(
            List<MatOfPoint> contours, Mat dst, boolean minRect,
            boolean displayContours, boolean displayAllBoxes, int thickness) {
        Mat hierarchy = new Mat();
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        //Rect[] boundRect = new Rect[contours.size()];
        RotatedRect[] boundRects = new RotatedRect[contours.size()];
        // Find Rects
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            if (minRect)
                boundRects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
            else
                boundRects[i] = rectToRotatedRect(Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray())));
        }

        // Display contours and boxes
        if (displayContours) {
            Scalar color = generateColor();
            drawContours(dst, contoursPoly, color, thickness);
        }
        if (displayAllBoxes) {
            Scalar color = generateColor();
            drawRotatedRects(dst, boundRects, color, thickness);
        }
        return boundRects;
    }

    public static void drawContours(Mat dst, MatOfPoint2f[] contoursPoly, Scalar color, int thickness) {
        ArrayList<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }
        for (int i = 0; i < contoursPoly.length; i++) {
            Imgproc.drawContours(dst, contoursPolyList, i, color, thickness);
        }
    }
    public static void drawRects(Mat dst, Rect[] rects, Scalar color, int thickness) {
        for (int i = 0; i < rects.length; i++) {
            Imgproc.rectangle(dst, rects[i], color, thickness);
        }
    }
    public static void drawRects(Mat dst, List<Rect> rects, Scalar color, int thickness) {
        for (Rect rect : rects) {
            Imgproc.rectangle(dst, rect, color, thickness);
        }
    }
    public static void drawRotatedRects(Mat dst, RotatedRect[] rects, Scalar color, int thickness) {
        for (int i = 0; i < rects.length; i++) {
            drawRotatedRectangle(dst, rects[i], color, thickness);
        }
    }
    public static void drawRotatedRects(Mat dst, ArrayList<RotatedRect> rects, Scalar color, int thickness) {
        for (RotatedRect rect : rects) {
            drawRotatedRectangle(dst, rect, color, thickness);
        }
    }

    public static void drawRotatedRectangle(Mat dst, RotatedRect rect, Scalar color, int thickness) {
        Point[] vertices = new Point[4];
        rect.points(vertices);
        List<MatOfPoint> boxContours = new ArrayList<>();
        boxContours.add(new MatOfPoint(vertices));
        Imgproc.drawContours(dst, boxContours, 0, color, thickness);
    }

    private static RotatedRect rectToRotatedRect(Rect rect) {
        RotatedRect rotatedRect = new RotatedRect();
        double x = rect.x, y = rect.y, h = rect.height, w = rect.width;
        rotatedRect.center = new Point(x + w / 2.0, y + h / 2.0);
        rotatedRect.size = rect.size();
        return rotatedRect;
    }

    public static Scalar generateColor() {
        Random rng = new Random(12345);
        return new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
    }
}
