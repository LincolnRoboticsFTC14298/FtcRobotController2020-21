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
import org.opencv.core.CvType;
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

import static org.firstinspires.ftc.teamcode.vision.VisionUtil.contains;

@Config
public class RingCountPipeline extends OpenCvPipeline {
    public static double SCORE_THRESHOLD = 3;
    public static int THICKNESS = 1;
    public static int RADIUS = 2;
    private Viewport viewport = Viewport.ANNOTATED;

    private static Rect croppedRect = new Rect(0, (int) ((1-.4)/2 * Vision.HEIGHT), Vision.WIDTH, (int) (Vision.HEIGHT*.4));
    private AnalysisRectMode analysisRectMode = AnalysisRectMode.WIDE;


    public static boolean watershed = false;

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
        SMALL(1.0 / 8,  1.0/8),
        WIDE(8.0 / 10, .9);

        private final Rect rect;
        private final int width = croppedRect.width;
        private final int height = croppedRect.height;

        AnalysisRectMode(double widthRatio, double heightRatio) {
            int w = (int) (width * widthRatio);
            int h = (int) (height * heightRatio);
            int x = (int) (width*(1 - widthRatio)/2), y = (int) (height*(1 - heightRatio)/2);
            rect = new Rect(x, y, w, h);
        }

        public Rect getRect() {
            return rect;
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

    @Override
    public void onViewportTapped() {
        int currentStageNum = viewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= Viewport.values().length)
        {
            nextStageNum = 0;
        }

        viewport = Viewport.values()[nextStageNum];

        if (!watershed) {
            switch (viewport) {
                case DIST1:
                case DIST2:
                case MARKERS:
                    viewport = Viewport.ANNOTATED;
                    break;
            }
        }
    }

    Mat rawImage = new Mat();
    Mat workingMat = new Mat();
    Mat croppedWorkingMat;
    Mat rawMask;
    Mat mask;
    Mat masked = new Mat();
    Mat markers = new Mat();
    Mat dist1 = new Mat();
    Mat dist2 = new Mat();
    Mat displayMat = new Mat();
    Mat displaySubmat;


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGBA2RGB);
        input.copyTo(rawImage);
        input.copyTo(workingMat);

        croppedWorkingMat = workingMat.submat(croppedRect);

        // MatOperator //
        rawMask = hsvRangeFilter.process(croppedWorkingMat);
        mask = morphologyOperator.process(rawMask);

        masked.release();
        masked = new Mat();
        croppedWorkingMat.copyTo(masked, mask);

        List<MatOfPoint> potentialContours = new ArrayList<>();

        // Segment //
        if (watershed) {
            markers.create(croppedWorkingMat.size(), CvType.CV_8UC3);
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
            // Must be within analysisRect to be analyzed
            if (contains(analysisRectMode.getRect(), ring.getBoundingRect())) {
                double score = calculateScore(ring);
                if (score <= SCORE_THRESHOLD) {
                    finalRings.add(ring);
                    finalContours.add(ring.getContour());
                    centers.add(ring.getCentroid());
                }
            }
        }

        rings = finalRings;

        // Draw contours //
        Imgproc.drawContours(croppedWorkingMat, potentialContours, -1, falseColor, THICKNESS);
        Imgproc.drawContours(croppedWorkingMat, finalContours, -1, foundColor, THICKNESS);

        // Draw centroids //
        for (Point center: centers) {
            Imgproc.circle(croppedWorkingMat, center, RADIUS, foundColor, THICKNESS);
        }

        // Draw rectangles //
        Imgproc.rectangle(croppedWorkingMat, analysisRectMode.getRect(), foundColor, THICKNESS);
        Imgproc.rectangle(workingMat, croppedRect, foundColor, THICKNESS/2);

        rawImage.copyTo(displayMat);
        displaySubmat = displayMat.submat(croppedRect);

        switch (viewport) {
            case RAW_IMAGE:
                rawImage.copyTo(displayMat);
                break;
            case RAW_MASK:
                Imgproc.cvtColor(rawMask, displaySubmat, Imgproc.COLOR_GRAY2RGB);
                break;
            case MASK:
                Imgproc.cvtColor(mask, displaySubmat, Imgproc.COLOR_GRAY2RGB);
                break;
            case MASKED:
                masked.copyTo(displaySubmat);
                break;
            case DIST1:
                if (watershed) {
                    Imgproc.cvtColor(dist1, displaySubmat, Imgproc.COLOR_GRAY2RGB);
                }
                else workingMat.copyTo(displayMat);
                break;
            case DIST2:
                if (watershed) {
                    Imgproc.cvtColor(dist2, displaySubmat, Imgproc.COLOR_GRAY2RGB);
                }
                else workingMat.copyTo(displayMat);
                break;
            case MARKERS:
                if (watershed)  {
                    markers.copyTo(displayMat);
                }
                else workingMat.copyTo(displayMat);
                break;
            default:
                workingMat.copyTo(displayMat);
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

    public void setWatershed(boolean watershed) {
        this.watershed = watershed;
        morphologyOperator.setClose(watershed);
    }

    public void setAnalysisRectMode(AnalysisRectMode analysisRectMode) {
        this.analysisRectMode = analysisRectMode;
    }

    public ArrayList<RingData> getRingData() {
        Collections.sort(rings, (r1, r2) -> (int) (r1.getContourArea() - r2.getContourArea()));
        Collections.reverse(rings);
        return rings;
    }

    public List<RingData> contoursToRingData(List<MatOfPoint> contours) {
        List<RingData> ringData = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            ringData.add(new RingData(contour));
        }
        return ringData;
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
}
