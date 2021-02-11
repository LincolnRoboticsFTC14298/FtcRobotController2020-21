package org.firstinspires.ftc.teamcode.vision.operators;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.vision.VisionUtil.generateColor;


@Config
public class SegmentationOperator {
    public static double binaryThresh = 40.0;
    public static Size openSize = new Size(25,7);
    public static double distThresh = 0.2;


    public SegmentationOperator() {
    }

    public List<MatOfPoint> process(Mat img, Mat dist1Dst, Mat dist2Dst, Mat dst) {
        // MODIFIED CODE FROM OPENCV EXAMPLE

        // Show source image
        Mat imgResult = img.clone();

        // convert back to 8bits gray scale
        imgResult.convertTo(imgResult, CvType.CV_8UC3);

        // Create binary image from source image
        Mat bw = new Mat();
        Imgproc.cvtColor(imgResult, bw, Imgproc.COLOR_RGB2GRAY);
        Imgproc.threshold(bw, bw, binaryThresh, 255, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);

        // Perform the distance transform algorithm
        Mat dist = new Mat();
        Imgproc.distanceTransform(bw, dist, Imgproc.DIST_L2, 3);
        bw.release();
        // Normalize the distance image for range = {0.0, 1.0}
        // so we can visualize and threshold it
        Core.normalize(dist, dist, 0.0, 1.0, Core.NORM_MINMAX);
        Mat distDisplayScaled = new Mat();
        Core.multiply(dist, new Scalar(255), distDisplayScaled);
        distDisplayScaled.convertTo(dist1Dst, CvType.CV_8U);
        distDisplayScaled.release();
        // Threshold to obtain the peaks
        // This will be the markers for the foreground objects
        Imgproc.threshold(dist, dist, distThresh, 1.0, Imgproc.THRESH_BINARY);
        // Dilate a bit the dist image
        // Originally dilate but changed open to insure distinct rings
        Mat kernel1 = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, openSize);
        Imgproc.morphologyEx(dist, dist, Imgproc.MORPH_OPEN, kernel1);
        kernel1.release();
        dist.convertTo(dist2Dst, CvType.CV_8U);
        Core.multiply(dist2Dst, new Scalar(255), dist2Dst);

        // Create the CV_8U version of the distance image
        // It is needed for findContours()
        Mat dist_8u = new Mat();
        dist.convertTo(dist_8u, CvType.CV_8U);
        // Find total markers
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(dist_8u, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        dist_8u.release();
        hierarchy.release();
        // Create the marker image for the watershed algorithm
        Mat markers = Mat.zeros(dist.size(), CvType.CV_32S);
        dist.release();
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
        markersScaled.release();
        markersDisplay.release();
        Imgproc.circle(markers, new Point(5, 5), 3, new Scalar(255, 255, 255), -1);


        // Perform the watershed algorithm
        imgResult.convertTo(imgResult, CvType.CV_8UC3);
        Imgproc.watershed(imgResult, markers);
        imgResult.release();
        Mat mark = Mat.zeros(markers.size(), CvType.CV_8U);
        markers.convertTo(mark, CvType.CV_8UC1);
        Core.bitwise_not(mark, mark);
        mark.release();
        // image looks like at that point
        // Generate random colors
        List<Scalar> colors = new ArrayList<>(contours.size());
        for (int i = 0; i < contours.size(); i++) {
            colors.add(generateColor());
        }

        // Create the result image
        //dst = Mat.zeros(markers.size(), CvType.CV_8UC3);
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

        List<MatOfPoint> finalContours = new ArrayList<>();
        Mat collage = Mat.zeros(markers.size(), CvType.CV_8UC1);
        byte[] collageData = new byte[(int) (collage.total())];
        for (int obj = 1; obj <= contours.size(); obj++) {
            Mat contourMat = Mat.zeros(markers.size(), CvType.CV_8UC1);
            byte[] contourData = new byte[(int) (contourMat.total())];
            contourMat.get(0, 0, contourData);

            for (int i = 0; i < markers.rows(); i++) {
                for (int j = 0; j < markers.cols(); j++) {
                    int index = markersData[i * markers.cols() + j];
                    if (index == obj && index <= contours.size()) {
                        contourData[i * dst.cols() + j] = (byte) (255);
                        collageData[i * dst.cols() + j] += (byte) (255);
                    } else {
                        contourData[i * dst.cols() + j] = 0;
                    }
                }
            }


            contourMat.put(0, 0, contourData);
            ArrayList<MatOfPoint> contourp = new ArrayList<>();
            Mat hierarchyp = new Mat();
            // Consider using chain approx none
            Imgproc.findContours(contourMat, contourp, hierarchyp, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            finalContours.addAll(contourp);
            contourMat.release();
            hierarchyp.release();
        }
        collage.put(0,0,collageData);
        markers.release();
        collage.release();
        //HighGui.imshow("si", fin);

//        Mat masked = new Mat();
//        Core.bitwise_not(fin, fin);
//        orig.copyTo(masked, fin);
//        Imgproc.drawContours(masked, finalContours, -1, new Scalar(0,255,0), 4);
//        HighGui.imshow("Help", masked);

        // Visualize the final image
        return finalContours;
    }
}
