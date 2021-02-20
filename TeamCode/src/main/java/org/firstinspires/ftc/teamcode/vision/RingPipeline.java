package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.VisionScorer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.operators.MorphologyOperator;
import org.firstinspires.ftc.teamcode.vision.operators.YCrCbRangeFilter;
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.vision.VisionUtil.contains;

@Config
public class RingPipeline extends OpenCvPipeline {
    public static double SCORE_THRESHOLD = 3;
    public static int THICKNESS = 5;
    public static int RADIUS = 8;
    private Viewport viewport = Viewport.ANNOTATED;

    private static Rect croppedRect = new Rect(0, (int) ((1-.4)/2 * Vision.HEIGHT), Vision.WIDTH, (int) (Vision.HEIGHT*.4));
    private AnalysisRectMode analysisRectMode = AnalysisRectMode.WIDE;

    private static final Scalar foundColor = new Scalar(0  , 255, 0  );
    private static final Scalar falseColor = new Scalar(255, 0  , 0  );

    private final ArrayList<VisionScorer> scorers = new ArrayList<>();

    private ArrayList<RingData> rings = new ArrayList<>();

    public AreaScorer areaScorer = new AreaScorer();
    public AspectRatioScorer aspectRatioSCorer = new AspectRatioScorer();
    public ExtentScorer extentScorer = new ExtentScorer();
    public SolidityScorer solidityScorer = new SolidityScorer();

    public YCrCbRangeFilter YCrCbRangeFilter = new YCrCbRangeFilter();
    public MorphologyOperator morphologyOperator = new MorphologyOperator();

    private Mat latestMat;

    public enum Viewport {
        RAW_IMAGE,
        RAW_MASK,
        MASK,
        MASKED,
        ANNOTATED
    }

    public enum AnalysisRectMode {
        SMALL(1.0 / 8,  1.0/8),
        WIDE(8.0 / 10, .9);

        private final Rect rect;

        AnalysisRectMode(double widthRatio, double heightRatio) {
            final int width = croppedRect.width;
            final int height = croppedRect.height;
            int w = (int) (width * widthRatio);
            int h = (int) (height * heightRatio);
            int x = (int) (width *(1 - widthRatio)/2), y = (int) (height *(1 - heightRatio)/2);
            rect = new Rect(x, y, w, h);
        }

        public Rect getRect() {
            return rect;
        }
    }


    public RingPipeline() {
        setAnalysisRectMode(analysisRectMode);

        scorers.add(areaScorer);
        scorers.add(aspectRatioSCorer);
        scorers.add(extentScorer);
        scorers.add(solidityScorer);
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
        rawMask = YCrCbRangeFilter.process(croppedWorkingMat);
        mask = morphologyOperator.process(rawMask);

        masked.release();
        masked = new Mat();
        croppedWorkingMat.copyTo(masked, mask);

        List<MatOfPoint> potentialContours = new ArrayList<>();

        // Contours //

        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, potentialContours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();

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
            default:
                workingMat.copyTo(displayMat);
                break;
        }
        latestMat = displayMat;
        return displayMat;
    }

    public Map<String, Object> getTelemetryData() {
        Map<String, Object> data = new HashMap<>();
        for (VisionScorer scorer : scorers) {
            scorer.updateTelemetry();
            data.putAll(scorer.getTelemetryData());
        }
        return data;
    }

    public void updateLogging() {
        for (VisionScorer scorer : scorers) {
            scorer.updateLogging();
        }
    }

    private double calculateScore(RingData ringData) {
        double score = 0.0;
        for (VisionScorer scorer : scorers) {
            score += scorer.score(ringData);
        }
        return score;
    }

    public void setClose(boolean close) {
        morphologyOperator.setClose(close);
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
    public void saveLatestMat(String filename) {
        saveMatToDisk(latestMat, filename);
    }
}
