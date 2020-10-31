package com.example.test.vision.newp;

import com.example.test.vision.newp.operators.HSVRangeFilter;
import com.example.test.vision.newp.operators.MorphologyOperator;
import com.example.test.vision.newp.operators.SegmentationOperator;
import com.example.test.vision.newp.scorers.AreaScorer;
import com.example.test.vision.newp.scorers.SolidityScorer;
import com.example.test.vision.newp.scorers.AspectRatioScorer;
import com.example.test.vision.newp.scorers.VisionScorer;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static com.example.test.vision.newp.VisionUtil.*;

public class RingCountPipeline {
    Object cam;
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

    public RingCountPipeline(Object cam, Viewport viewport) {
        this.cam = cam;
        this.viewport = viewport;

        scorers.add(areaScorer);
        scorers.add(solidityScorer);
        scorers.add(aspectRatioSCorer);
    }


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

    public int getNumRingsFound() {
        return rings;
    }
}
