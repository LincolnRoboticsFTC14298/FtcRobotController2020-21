package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.operators.HSVFilter;
import org.firstinspires.ftc.teamcode.vision.scorers.VisionScorer;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.vision.VisionUtil.*;

public class NewRingCountPipeline extends OpenCvPipeline {
    OpenCvInternalCamera2 cam;
    private boolean viewportPaused = false;
    private static final double SCORE_THRESHOLD = 700;
    private static final int THICKNESS = 4;
    private Viewport viewport;
    private static Scalar displayColor = new Scalar(0,255,0);
    private int rings = 0;

    private ArrayList<VisionScorer> scorers = new ArrayList<>();

    private HSVFilter hsvFilter = new HSVFilter(new Scalar(10, 150, 150), new Scalar(20, 255, 255));

    public enum Viewport {
        RAW_IMAGE,
        MASK,
        MARKERS
    }

    public NewRingCountPipeline(OpenCvInternalCamera2 cam, Viewport viewport) {
        this.cam = cam;
        this.viewport = viewport;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat rawImage = input.clone();
        Mat workingMat = input.clone();

        // Crop //
        int x = 3 * input.width() / 10, y = 3 * input.height() / 10, w = input.width() - 2 * x, h = input.height() - 2 * y;
        Rect rectCrop = new Rect(x, y, w, h);
        Mat croppedWorkingMat = workingMat.submat(rectCrop); // For submat, modifying sub modifies that rect region in parent
        Mat markers = croppedWorkingMat.clone();

        // Filter //
        Mat mask = hsvFilter.process(croppedWorkingMat);

        // Segment //
        List<MatOfPoint> contours = segmentation(mask, markers);
        Rect[] potentialRingRects = getBoundingRectangles(contours);

        // Score and Threshold //
        ArrayList<Rect> finalRingRects = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            double score = calculateScore(potentialRingRects[i], contours.get(i));
            if (score <= SCORE_THRESHOLD) {
                finalRingRects.add(potentialRingRects[i]);
            }
        }

        rings = finalRingRects.size();

        // Draw rect on input //
        drawRectangles(croppedWorkingMat, finalRingRects, displayColor, THICKNESS);
        Imgproc.rectangle(workingMat, rectCrop, displayColor, THICKNESS);

        Mat displayMat = input.clone();
        switch (viewport) {
            case RAW_IMAGE:
                displayMat = rawImage;
                break;
            case MASK:
                displayMat = mask;
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

    private double calculateScore(Rect rect, MatOfPoint contour) {
        double score = 0.0;
        for (VisionScorer scorer : scorers) {
            score += scorer.score(rect, contour);
        }
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
