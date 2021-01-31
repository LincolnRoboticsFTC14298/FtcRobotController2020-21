package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.VisionScorer;
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
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class RingCountPipeline extends OpenCvPipeline {
    private final OpenCvInternalCamera2 cam;

    private boolean viewportPaused = false;
    public static double SCORE_THRESHOLD = 3;
    public static int THICKNESS = 4;
    public static int RADIUS = 8;
    private Viewport viewport = Viewport.ANNOTATED;
    private CroppedRectMode croppedRectMode = CroppedRectMode.WIDE;
    private boolean watershed = false;

    private static final Scalar foundColor = new Scalar(0  , 255, 0  );
    private static final Scalar falseColor = new Scalar(0  , 0  , 255);

    private final ArrayList<VisionScorer> scorers = new ArrayList<>();
    public double totalWeight = 0;

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

    public enum CroppedRectMode {
        SMALL(3.0 / 10, 3.0 / 10),
        WIDE(5.0 / 10, 3.0 / 10);

        public double widthRatio;
        public double heightRatio;

        CroppedRectMode(double widthRatio, double heightRatio) {
            this.widthRatio = widthRatio;
            this.heightRatio = heightRatio;
        }
    }


    public RingCountPipeline(OpenCvInternalCamera2 cam) {
        this.cam = cam;

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

    @Override
    public Mat processFrame(Mat input) {
        // TODO: investigate skipping segmentation and not doing morph_close
        Mat rawImage = input.clone();
        Mat workingMat = input.clone();

        // Crop //
        int w = (int) (input.width() * croppedRectMode.widthRatio);
        int h = (int) (input.height() * croppedRectMode.heightRatio);
        int x = input.width()/2 - w/2, y = input.height()/2 - h/2;
        Rect rectCrop = new Rect(x, y, w, h);
        Mat croppedWorkingMat = workingMat.submat(rectCrop); // For submat, modifying sub modifies that rect region in parent


        // MatOperator //
        Mat rawMask = hsvRangeFilter.process(croppedWorkingMat);
        Mat mask = morphologyOperator.process(rawMask);

        Mat masked = new Mat();
        croppedWorkingMat.copyTo(masked, mask);


        // Segment //
        Mat markers = croppedWorkingMat.clone();
        Mat dist1 = croppedWorkingMat.clone();
        Mat dist2 = croppedWorkingMat.clone();

        List<MatOfPoint> potentialContours = new ArrayList<>();


        if (watershed) {
            potentialContours = segmentationOperator.process(masked, dist1, dist2, markers);
        } else {
            Mat hierarchy = new Mat();
            // Consider using chain approx none
            Imgproc.findContours(mask, potentialContours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        }

        List<RingData> potentialRings = contoursToRingData(potentialContours);

        // Score and Threshold //
        ArrayList<RingData> finalRings = new ArrayList<>();
        ArrayList<MatOfPoint> finalContours = new ArrayList<>();
        ArrayList<Point> centers = new ArrayList<>();

        for (int i = 0; i < potentialRings.size(); i++) {
            double score = calculateScore(potentialRings.get(i));
            if (score <= SCORE_THRESHOLD) {
                //System.out.println("Final score: " + score);
                finalRings.add(potentialRings.get(i));
                finalContours.add(potentialRings.get(i).contour);
                centers.add(potentialRings.get(i).centroid);
            }
        }

        rings = finalRings;
        Collections.sort(rings, (r1, r2) -> (int) (r1.contourArea - r2.contourArea));
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

        Imgproc.rectangle(workingMat, rectCrop, foundColor, THICKNESS);

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
        viewportPaused = !viewportPaused;

        if(viewportPaused) {
            cam.pauseViewport();
        }
        else {
            cam.resumeViewport();
        }
    }

    public void setWatershed(boolean watershed) {
        this.watershed = watershed;
        morphologyOperator.setClose(watershed);
    }

    public void setCroppedRectMode(CroppedRectMode croppedRectMode) {
        this.croppedRectMode = croppedRectMode;
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
