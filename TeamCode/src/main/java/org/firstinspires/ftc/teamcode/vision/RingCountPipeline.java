package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.operators.HSVRangeFilter;
import org.firstinspires.ftc.teamcode.vision.operators.MorphologyOperator;
import org.firstinspires.ftc.teamcode.vision.operators.SegmentationOperator;
import org.firstinspires.ftc.teamcode.vision.scorers.AspectRatioScorer;
import org.firstinspires.ftc.teamcode.vision.scorers.SolidityScorer;
import org.firstinspires.ftc.teamcode.vision.scorers.VisionScorer;
import org.firstinspires.ftc.teamcode.vision.scorers.AreaScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.vision.VisionUtil.getBoundingRects;
import static org.firstinspires.ftc.teamcode.vision.VisionUtil.rectCenter;

public class RingCountPipeline extends OpenCvPipeline {
    private OpenCvInternalCamera2 cam;
    private boolean viewportPaused = false;
    private static final double SCORE_THRESHOLD = 3;
    private static final int THICKNESS = 4;
    private Viewport viewport;
    private static Scalar foundColor = new Scalar(0,255,0);
    private static Scalar falseColor = new Scalar(0,0,255);
    private int rings = 0;

    private ArrayList<VisionScorer> scorers = new ArrayList<>();

    private AreaScorer areaScorer = new AreaScorer();
    private SolidityScorer solidityScorer = new SolidityScorer();
    private AspectRatioScorer aspectRatioSCorer = new AspectRatioScorer();

    private HSVRangeFilter hsvRangeFilter = new HSVRangeFilter();
    private MorphologyOperator morphologyOperator = new MorphologyOperator();
    private SegmentationOperator segmentationOperator = new SegmentationOperator();

    public enum Viewport {
        RAW_IMAGE,
        RAW_MASK,
        MASK,
        MASKED,
        DIST1,
        DIST2,
        MARKERS,
        ANNOTATED
    }

    public RingCountPipeline(OpenCvInternalCamera2 cam, Viewport viewport) {
        this.cam = cam;
        this.viewport = viewport;

        scorers.add(areaScorer);
        scorers.add(solidityScorer);
        scorers.add(aspectRatioSCorer);
    }

    public Viewport getViewport() {
        return viewport;
    }
    public void setViewport(Viewport viewport) {
        this.viewport = viewport;
    }


    @Override
    public Mat processFrame(Mat input) {
        // TODO: investigate skipping segmentation and not doing morph_close
        Mat rawImage = input.clone();
        Mat workingMat = input.clone();

        // Crop //
        int x = 3 * input.width() / 10, y = 3 * input.height() / 10, w = input.width() - 2 * x, h = input.height() - 2 * y;
        Rect rectCrop = new Rect(x, y, w, h);
        Mat croppedWorkingMat = workingMat.submat(rectCrop); // For submat, modifying sub modifies that rect region in parent


        // MatOperator //
        Mat rawMask = hsvRangeFilter.process(croppedWorkingMat);
        Mat mask = morphologyOperator.process(rawMask);

        Mat masked = new Mat();
        croppedWorkingMat.copyTo(masked, mask);


        // Segment //
        //ArrayList<MatOfPoint> potentialContours = new ArrayList<>();
        //Mat hierarchy = new Mat();
        // Consider using chain approx none
        //Imgproc.findContours(mask, potentialContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Mat markers = croppedWorkingMat.clone();
        Mat dist1 = croppedWorkingMat.clone();
        Mat dist2 = croppedWorkingMat.clone();
        List<MatOfPoint> potentialContours = segmentationOperator.process(masked, dist1, dist2, markers);
        Rect[] potentialRects = getBoundingRects(potentialContours);

        // Score and Threshold //
        ArrayList<Rect> finalRects = new ArrayList<>();
        ArrayList<MatOfPoint> finalContours = new ArrayList<>();
        ArrayList<Point> centers = new ArrayList<>();
        for (int i = 0; i < potentialContours.size(); i++) {
            double score = calculateScore(potentialContours.get(i));
            if (score <= SCORE_THRESHOLD) {
                //System.out.println("Final score: " + score);
                finalRects.add(potentialRects[i]);
                finalContours.add(potentialContours.get(i));
                centers.add(rectCenter(potentialRects[i]));
            }
        }

        rings = finalRects.size();
        if (rings == 3) rings = 4;
        //System.out.println(rings);

        // Draw contours //
        Imgproc.drawContours(croppedWorkingMat, potentialContours, -1, falseColor, THICKNESS);
        Imgproc.drawContours(croppedWorkingMat, finalContours, -1, foundColor, THICKNESS);

        // Draw rect on input //
        //drawRectangles(croppedWorkingMat, potentialRects, falseColor, THICKNESS); // Wrong rings will be red
        //drawRectangles(croppedWorkingMat, finalRects, foundColor, THICKNESS);


        Imgproc.rectangle(workingMat, rectCrop, foundColor, THICKNESS);

        Mat displayMat = input.clone();
        switch (viewport) {
            case RAW_IMAGE:
                displayMat = rawImage;
                break;
            case RAW_MASK:
                displayMat = rawMask;
                break;
            case MASK:
                displayMat = mask;
                break;
            case MASKED:
                displayMat = masked;
                break;
            case DIST1:
                displayMat = dist1;
                break;
            case DIST2:
                displayMat = dist2;
                break;
            case MARKERS:
                displayMat = markers;
                break;
            default:
                displayMat = workingMat;
                break;
        }
        return displayMat;
    }

    private double calculateScore(MatOfPoint contour) {
        double score = 0.0;
        for (VisionScorer scorer : scorers) {
            score += scorer.score(contour);
        }
        //System.out.println(score);
        return score;
    }

    @Override
    public void onViewportTapped() {
        viewportPaused = !viewportPaused;

        if(viewportPaused) {
            cam.pauseViewport();
        }
        else {
            cam.resumeViewport();
        }
    }

    public int getNumRingsFound() {
        return rings;
    }
}
