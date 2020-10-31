package org.firstinspires.ftc.teamcode.vision.old;

import org.opencv.core.*;

import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.firstinspires.ftc.teamcode.vision.VisionUtil.*;

public class CameraRingDetection {
    // Initiating Imagecodecs
    public static Imgcodecs imageCodecs = new Imgcodecs();

    public static final Scalar LOWER_THRESH = new Scalar(10, 150, 150);
    public static final Scalar UPPER_THRESH = new Scalar(20, 255, 255);


    private static Random rng = new Random(12345);

    public static Mat mask(Mat img, Size size, boolean morphOpen) {
        // Create Mask //
        Mat mask = new Mat();
        Imgproc.cvtColor(img, mask, Imgproc.COLOR_BGR2HSV);
        Core.inRange(mask, LOWER_THRESH, UPPER_THRESH, mask);


        // Denoising //
        //Imgproc.GaussianBlur(mask, mask, new Size(101,101), 10);

        //Closes gaps
        Mat se = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, size);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, se);

        if (morphOpen) {
            // Remove isolated pixels using morphological operations
            // Preparing the kernel matrix object
            Mat kernel = Mat.ones(5, 5, CvType.CV_32F);
            // Applying dilate on the Image
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        }

        return mask;
    }

    public static int ringCountUsingArea(Mat img, double[] areaThresh,
                                         int thickness, boolean debug) {
        //Calculate area
        double area = ringsArea(img, thickness, debug);
        // Display image and show ring count
        if (area > areaThresh[1]) return 4;
        else if (area > areaThresh[0]) return 1;
        else if (area < areaThresh[1]) return 0;
        else return -1;
    }

    private static double ringsArea(Mat img, int thickness, boolean debug) {
        // Cropped region is for analysis
        // (x,y) is top left corner
        int x = 3 * img.width() / 10, y = 3 * img.height() / 10, w = img.width() - 2 * x, h = img.height() - 2 * y;
        Rect rectCrop = new Rect(x, y, w, h);
        Mat croppedImg = img.submat(rectCrop); // For submat, modifying sub modifies that rect region in parent

        // Find mask
        Mat croppedMask = mask(croppedImg, new Size(30, 30), true);



        // Showing mask
        Mat masked = new Mat();
        croppedImg.copyTo(masked, croppedMask);

        // Find Contours using canny
        ArrayList<MatOfPoint> contours = findContours(croppedMask, -0.5, 0.5);

        // Set minRect to false if you want non rotated rect
        RotatedRect[] boundingRect = getBoundingRotatedRectangles(contours, croppedImg, debug, debug, 2);
        double largestArea = 0;
        RotatedRect largestRect = new RotatedRect();
        for (int i = 0; i < contours.size(); i++) {
            double area = area(boundingRect[i]);
            if (debug) System.out.println(area + " " + i);
            if (area > largestArea) {
                largestArea = area;
                largestRect = boundingRect[i];
            }
        }

        // Display box
        //Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
        drawRotatedRectangle(croppedImg, largestRect, new Scalar(0, 255, 0), thickness);

        Imgproc.rectangle(img, rectCrop, new Scalar(0, 255, 0), thickness);

        if (debug) System.out.println("Largest Area " + largestArea);
        return largestArea;
    }

    public static int ringCountUsingSegmentation(Mat img, double areaThresh,
                                                 int thickness, boolean debug) {
        // Cropped region is for analysis
        // (x,y) is top left corner
        int x = 3 * img.width() / 10, y = 3 * img.height() / 10, w = img.width() - 2 * x, h = img.height() - 2 * y;
        Rect rectCrop = new Rect(x, y, w, h);
        Mat croppedImg = img.submat(rectCrop); // For submat, modifying sub modifies that rect region in parent

        // Find mask
        Mat croppedMask = mask(croppedImg, new Size(30, 30), true);

        Mat masked = new Mat();
        croppedImg.copyTo(masked, croppedMask);

        // Segmentation
        List<MatOfPoint> contours = segmentation(masked, debug);

        // If countour area > threshold, it is a ring

        RotatedRect[] boundingRect = getBoundingRotatedRectangles(contours, croppedImg, debug, debug, 2);
        int numRings = 0;
        ArrayList<RotatedRect> rotatedRects = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            RotatedRect rect = boundingRect[i];
            double area = area(rect);
            if (area > areaThresh) {
                numRings++;
                rotatedRects.add(rect);
            }
        }

        // Display boxes
        drawRotatedRectangles(croppedImg, rotatedRects, new Scalar(0, 255, 0), thickness);


        // Display where they are
        Imgproc.rectangle(img, rectCrop, new Scalar(0, 0, 255), thickness);
        return numRings;
    }

    private static List<MatOfPoint> segmentation(Mat src, boolean debug) {
        // MODIFIED CODE FROM OPENCV EXAMPLE

        // Show source image
        //HighGui.imshow("Source Image", src);
        Mat imgResult = src.clone();

        // convert back to 8bits gray scale
        imgResult.convertTo(imgResult, CvType.CV_8UC3);
        // imshow( "Laplace Filtered Image", imgLaplacian );
        //HighGui.imshow("New Sharped Image", imgResult);

        // Create binary image from source image
        Mat bw = new Mat();
        Imgproc.cvtColor(imgResult, bw, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(bw, bw, 40, 255, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);

        // Perform the distance transform algorithm
        Mat dist = new Mat();
        Imgproc.distanceTransform(bw, dist, Imgproc.DIST_L2, 3);
        // Normalize the distance image for range = {0.0, 1.0}
        // so we can visualize and threshold it
        Core.normalize(dist, dist, 0.0, 1.0, Core.NORM_MINMAX);
        Mat distDisplayScaled = new Mat();
        Core.multiply(dist, new Scalar(255), distDisplayScaled);
        Mat distDisplay = new Mat();
        distDisplayScaled.convertTo(distDisplay, CvType.CV_8U);
        // Threshold to obtain the peaks
        // This will be the markers for the foreground objects
        Imgproc.threshold(dist, dist, 0.2, 1.0, Imgproc.THRESH_BINARY);
        // Dilate a bit the dist image
        Mat kernel1 = Mat.ones(3, 3, CvType.CV_8U);
        Imgproc.dilate(dist, dist, kernel1);
        Mat distDisplay2 = new Mat();
        dist.convertTo(distDisplay2, CvType.CV_8U);
        Core.multiply(distDisplay2, new Scalar(255), distDisplay2);

        // Create the CV_8U version of the distance image
        // It is needed for findContours()
        Mat dist_8u = new Mat();
        dist.convertTo(dist_8u, CvType.CV_8U);
        // Find total markers
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(dist_8u, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        // Create the marker image for the watershed algorithm
        Mat markers = Mat.zeros(dist.size(), CvType.CV_32S);
        // Draw the foreground markers
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(markers, contours, i, new Scalar(i + 1), -1);
        }

        // Draw the background marker
        Mat markersScaled = new Mat();
        markers.convertTo(markersScaled, CvType.CV_32F);
        Core.normalize(markersScaled, markersScaled, 0.0, 255.0, Core.NORM_MINMAX);
        Imgproc.circle(markersScaled, new Point(5, 5), 3, new Scalar(255, 255, 255), -1);
        Mat markersDisplay = new Mat();
        markersScaled.convertTo(markersDisplay, CvType.CV_8U);
        Imgproc.circle(markers, new Point(5, 5), 3, new Scalar(255, 255, 255), -1);


        // Perform the watershed algorithm
        Imgproc.watershed(imgResult, markers);
        Mat mark = Mat.zeros(markers.size(), CvType.CV_8U);
        markers.convertTo(mark, CvType.CV_8UC1);
        Core.bitwise_not(mark, mark);
        // image looks like at that point
        // Generate random colors
        Random rng = new Random(12345);
        List<Scalar> colors = new ArrayList<>(contours.size());
        for (int i = 0; i < contours.size(); i++) {
            int b = rng.nextInt(256);
            int g = rng.nextInt(256);
            int r = rng.nextInt(256);
            colors.add(new Scalar(b, g, r));
        }


        // Create the result image
        Mat dst = Mat.zeros(markers.size(), CvType.CV_8UC3);
        byte[] dstData = new byte[(int) (dst.total() * dst.channels())];
        dst.get(0, 0, dstData);
        // Fill labeled objects with random colors
        int[] markersData = new int[(int) (markers.total() * markers.channels())];
        markers.get(0, 0, markersData);
        for (int i = 0; i < markers.rows(); i++) {
            for (int j = 0; j < markers.cols(); j++) {
                int index = markersData[i * markers.cols() + j];
                if (index > 0 && index <= contours.size()) {
                    dstData[(i * dst.cols() + j) * 3 + 0] = (byte) colors.get(index - 1).val[0];
                    dstData[(i * dst.cols() + j) * 3 + 1] = (byte) colors.get(index - 1).val[1];
                    dstData[(i * dst.cols() + j) * 3 + 2] = (byte) colors.get(index - 1).val[2];
                } else {
                    dstData[(i * dst.cols() + j) * 3 + 0] = 0;
                    dstData[(i * dst.cols() + j) * 3 + 1] = 0;
                    dstData[(i * dst.cols() + j) * 3 + 2] = 0;
                }
            }
        }
        dst.put(0, 0, dstData);
        // Visualize the final image
        return contours;
    }


    private static double area(RotatedRect rect) {
        Point[] points = new Point[4];
        rect.points(points);
        double area = 1;

        //Multiply by all lengths (a^2 = l^2 * w^2), then take square root
        for (int i = 0; i < 4; i++) {
            Point p1 = points[i];
            Point p2 = points[(i + 1) % 4];
            area *= Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
        }
        return Math.sqrt(area);
    }
}
