package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Vision.HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.Vision.WIDTH;

public class VisionUtil {

    public static Point rectCenter(Rect rect) {
        double x = rect.x + rect.width / 2.0;
        double y = rect.y + rect.height / 2.0;
        return new Point(x, y);
    }

    public static RotatedRect[] getBoundingRotatedRects(List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        RotatedRect[] boundRects = new RotatedRect[contours.size()];
        // Find Rects
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
        }
        return boundRects;
    }

    public static void drawRotatedRectangle(Mat dst, RotatedRect rect, Scalar color, int thickness) {
        Point[] vertices = new Point[4];
        rect.points(vertices);
        List<MatOfPoint> boxContours = new ArrayList<>();
        boxContours.add(new MatOfPoint(vertices));
        Imgproc.drawContours(dst, boxContours, 0, color, thickness);
    }

    public static MatOfPoint2f pickPoints(MatOfPoint points, MatOfInt indices) {
        Point[] pickedPoints = new Point[indices.rows()];
        int newRow = 0;
        for (int index : indices.toArray()) {
            pickedPoints[newRow++] = new Point(points.get(index, 0));
        }
        return new MatOfPoint2f(pickedPoints);
    }

    /*
     * Return true if r1 contains r2
     */
    public static boolean contains(Rect r1, Rect r2) {
        if (r1.contains(r2.tl()) && r1.contains(r2.br()))
            return true;
        else
            return false;
    }

    /*
     * Converts x from (0, w) -> (1, -1) and
     * converts y from (0, h) -> (1, -1)
     */
    public static Point normalizePoint(Point p) {
        double x = -(p.x - WIDTH / 2.0) / (WIDTH / 2.0);
        double y = -(p.y - HEIGHT / 2.0) / (HEIGHT / 2.0);
        return new Point(x, y);
    }

    public static Size normalizeSize(Size s) {
        double w = s.width / WIDTH;
        double h = s.height / HEIGHT;
        return new Size(w, h);
    }

    public static Rect normalizeRect(Rect r) {
        return new Rect(normalizePoint(r.tl()), normalizeSize(r.size()));
    }

    public static double normalizeArea(double a) {
        return a / (WIDTH * HEIGHT);
    }

    private static final Random rng = new Random(12345);
    public static Scalar generateColor() {
        return new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
    }
}
