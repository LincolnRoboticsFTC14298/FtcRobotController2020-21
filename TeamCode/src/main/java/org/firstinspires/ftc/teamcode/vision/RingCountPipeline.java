package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.VisionScorer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.operators.HSVRangeFilter;
import org.firstinspires.ftc.teamcode.vision.operators.MorphologyOperator;
import org.firstinspires.ftc.teamcode.vision.operators.SegmentationOperator;
import org.firstinspires.ftc.teamcode.vision.scorers.AreaScorer;
import org.firstinspires.ftc.teamcode.vision.scorers.AspectRatioScorer;
import org.firstinspires.ftc.teamcode.vision.scorers.ExtentScorer;
import org.firstinspires.ftc.teamcode.vision.scorers.SolidityScorer;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class RingCountPipeline extends OpenCvPipeline {
    public static double SCORE_THRESHOLD = 3;
    public static int THICKNESS = 3;
    public static int RADIUS = 8;
    private Viewport viewport = Viewport.ANNOTATED;
    private AnalysisRectMode analysisRectMode = AnalysisRectMode.WIDE;
    private Rect analysisRect;
    private Rect croppedRect = new Rect(0, Vision.HEIGHT/3, Vision.WIDTH, Vision.HEIGHT/3);
    private boolean watershed = false;

    private static final Scalar foundColor = new Scalar(0  , 255, 0  );
    private static final Scalar falseColor = new Scalar(255, 0  , 0  );

    private final ArrayList<VisionScorer> scorers = new ArrayList<>();
    private double totalWeight = 0;

    private ArrayList<RingData> rings = new ArrayList<>();

    public AreaScorer areaScorer = new AreaScorer();
    public AspectRatioScorer aspectRatioSCorer = new AspectRatioScorer();
    public ExtentScorer extentScorer = new ExtentScorer();
    public SolidityScorer solidityScorer = new SolidityScorer();


    public HSVRangeFilter hsvRangeFilter = new HSVRangeFilter();
    public MorphologyOperator morphologyOperator = new MorphologyOperator();
    public SegmentationOperator segmentationOperator = new SegmentationOperator();

    private Mat latestMat;

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

    public enum AnalysisRectMode {
        SMALL(3.0 / 10, 3.0 / 10),
        WIDE(5.0 / 10, 3.0 / 10);

        public double widthRatio;
        public double heightRatio;

        AnalysisRectMode(double widthRatio, double heightRatio) {
            this.widthRatio = widthRatio;
            this.heightRatio = heightRatio;
        }
    }


    public RingCountPipeline() {
        setAnalysisRectMode(analysisRectMode);

        scorers.add(areaScorer);
        scorers.add(aspectRatioSCorer);
        scorers.add(extentScorer);
        scorers.add(solidityScorer);

        for (VisionScorer scorer : scorers) {
            totalWeight += scorer.weight;
        }
    }



    public Viewport getViewport() {
        return viewport;
    }
    public void setViewport(Viewport viewport) {
        this.viewport = viewport;
    }
    public Mat getLatestMat() {
        return latestMat;
    }
    public void saveLatestMat(String filename) {
        saveMatToDisk(latestMat, filename);
    }

    Mat rawImage = new Mat();
    Mat workingMat = new Mat();
    Mat croppedWorkingMat = new Mat(); // For submat, modifying sub modifies that rect region in parent
    Mat rawMask = new Mat();
    Mat mask = new Mat();
    Mat masked = new Mat();
    Mat markers = new Mat();
    Mat dist1 = new Mat();
    Mat dist2 = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        //Imgproc.cvtColor(input,input,Imgproc.COLOR_RGBA2RGB);
        input.copyTo(rawImage);
        input.copyTo(workingMat);

        croppedWorkingMat = workingMat.submat(croppedRect);

        // MatOperator //
        rawMask = hsvRangeFilter.process(croppedWorkingMat);
        mask = morphologyOperator.process(rawMask);

        croppedWorkingMat.copyTo(masked, mask);

        List<MatOfPoint> potentialContours = new ArrayList<>();

        // Segment //
        if (watershed) {
            potentialContours = segmentationOperator.process(masked, dist1, dist2, markers);
        } else {
            Mat hierarchy = new Mat();
            // Consider using chain approx none
            Imgproc.findContours(mask, potentialContours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            hierarchy.release();
        }

        List<RingData> potentialRings = contoursToRingData(potentialContours);

        ArrayList<RingData> finalRings = new ArrayList<>();
        ArrayList<MatOfPoint> finalContours = new ArrayList<>();
        ArrayList<Point> centers = new ArrayList<>();

        // Score and Threshold //
        for (int i = 0; i < potentialRings.size(); i++) {
            RingData ring = potentialRings.get(i);
            // TODO: FIX!!!
            // Must be within analysisRect to be analyzed
            //if (contains(analysisRect, ring.getBoundingRect())) {
                double score = calculateScore(ring);
                if (score <= SCORE_THRESHOLD) {
                    finalRings.add(ring);
                    finalContours.add(ring.getContour());
                    centers.add(ring.getCentroid());
                }
            //}
        }

        rings = finalRings;
        Collections.sort(rings, (r1, r2) -> (int) (r1.getContourArea() - r2.getContourArea()));
        Collections.reverse(rings);

        // Draw contours //
        Imgproc.drawContours(croppedWorkingMat, potentialContours, -1, falseColor, THICKNESS);
        Imgproc.drawContours(croppedWorkingMat, finalContours, -1, foundColor, THICKNESS);

        // Draw contour centers //
        for (Point center: centers) {
            Imgproc.circle(croppedWorkingMat, center, RADIUS, foundColor, THICKNESS);
        }

        // Draw rect on input //
        //drawRectangles(croppedWorkingMat, potentialRects, falseColor, THICKNESS); // Wrong rings will be red
        //drawRectangles(croppedWorkingMat, finalRects, foundColor, THICKNESS);

        Imgproc.rectangle(workingMat, analysisRect, foundColor, THICKNESS);
        Imgproc.rectangle(workingMat, croppedRect, foundColor, THICKNESS);

        Mat displayMat;
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
                if (watershed) displayMat = dist1;
                else displayMat = workingMat;
                break;
            case DIST2:
                if (watershed) displayMat = dist2;
                else displayMat = workingMat;
                break;
            case MARKERS:
                if (watershed) displayMat = markers;
                else displayMat = workingMat;
                break;
            default:
                displayMat = workingMat;
                break;
        }
        latestMat = displayMat;
        return displayMat;
    }

    private double calculateScore(RingData ringData) {
        double score = 0.0;
        for (VisionScorer scorer : scorers) {
            score += scorer.score(ringData);
        }
        return score / totalWeight;
    }

    @Override
    public void onViewportTapped() {
        int currentStageNum = viewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= Viewport.values().length)
        {
            nextStageNum = 0;
        }

        viewport = Viewport.values()[nextStageNum];
    }

    public void setWatershed(boolean watershed) {
        this.watershed = watershed;
        morphologyOperator.setClose(watershed);
    }

    public void setAnalysisRectMode(AnalysisRectMode analysisRectMode) {
        this.analysisRectMode = analysisRectMode;
        int w = (int) (rawImage.width() * analysisRectMode.widthRatio);
        int h = (int) (rawImage.height() * analysisRectMode.heightRatio);
        int x = rawImage.width()/2 - w/2, y = rawImage.height()/2 - h/2;
        analysisRect = new Rect(x, y, w, h);
    }

    public ArrayList<RingData> getRingData() {
        return rings;
    }

    public List<RingData> contoursToRingData(List<MatOfPoint> contours) {
        List<RingData> ringData = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            ringData.add(new RingData(contour));
        }
        return ringData;
    }
}
