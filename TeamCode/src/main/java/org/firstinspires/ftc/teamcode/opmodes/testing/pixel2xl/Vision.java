package org.firstinspires.ftc.teamcode.opmodes.testing.pixel2xl;

import android.util.Log;

import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline;
import org.firstinspires.ftc.teamcode.vision.RingData;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

import static org.firstinspires.ftc.teamcode.util.Ring.RING_DIAMETER;

public class Vision extends AbstractSubsystem {
    public static int WIDTH = 1280;
    public static int HEIGHT = 720;
    public static final double FOV_X = Math.toRadians(76), FOV_Y = Math.toRadians(21); // radians
    public static double FUDGE_FACTOR_Y = 1, FUDGE_FACTOR_X = 1;

    public static int oneRingHeight = 10;
    public static int zeroRingHeight = 10;

    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private OpenCvInternalCamera camera;
    private RingCountPipeline ringCountPipeline;

    private List<RingData> ringData;

    private Localizer localizer;

    public Vision(HardwareMap hardwareMap, Localizer localizer) {
        super("Vision");
        this.localizer = localizer;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        ringCountPipeline = new RingCountPipeline();
        camera.setPipeline(ringCountPipeline);

        camera.openCameraDeviceAsync(
                () -> {
                    camera.setViewportRenderingPolicy(OpenCvInternalCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                    camera.setViewportRenderer(OpenCvInternalCamera.ViewportRenderer.GPU_ACCELERATED);
                    camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }
        );
    }

    public Vision(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        camera.stopStreaming();
    }

    @Override
    public void updateTelemetry() {
        if (ringData != null) telemetry.put("Number of Rings", ringData.size());
        telemetry.put("Viewport", getViewport());
        telemetry.putAll(ringCountPipeline.getTelemetryData());
    }

    @Override
    public void updateLogging() {
        ringCountPipeline.updateLogging();
    }

    public void analyze() {
        ringData = ringCountPipeline.getRingData();
    }

    /*
     * Frame of reference: camera is center and axis same as cameras
     */
    public Vector3D getRingCameraLocalPosition(RingData ring) {
        double ratio = RING_DIAMETER / ring.getNormalizedBoxSize().width;
        double x = FUDGE_FACTOR_X * ratio / Math.tan(FOV_X / 2.0);
        double y = FUDGE_FACTOR_Y * ratio * ring.getNormalizedCentroid().x;
        double z = Math.tan(FOV_Y / 2.0) * ring.getNormalizedCentroid().y * x;
        return new Vector3D(x, y, z);
    }

    public void resumeViewport() {
        camera.resumeViewport();
    }
    public void pauseViewport() {
        camera.pauseViewport();
    }

    // Getters and Setters //
    public List<RingData> getRingData() {
        return ringData;
    }
    public int getNumRings() {
        if (ringData != null) {
            return ringData.size();
        } else {
            Log.e("Vision", "Ring Data is null");
            return 0;
        }
    }

    public int getCenterStackSize() {
        // TODO: Center stack may not be considered a stack
        RingData centerRing = ringData.get(0);
        if (centerRing != null) {
            double height = centerRing.getBoxSize().height;
            if (height > oneRingHeight) {
                return 4;
            } else if (height > zeroRingHeight) {
                return 1;
            }
        }
        return 0;
    }

    public RingCountPipeline.Viewport getViewport() {
        return ringCountPipeline.getViewport();
    }
    public void setViewport(RingCountPipeline.Viewport viewport) {
        ringCountPipeline.setViewport(viewport);
    }

    public void setWatershed(boolean watershed) {
        ringCountPipeline.setWatershed(watershed);
    }
    public void setCroppedRectMode(RingCountPipeline.AnalysisRectMode analysisRectMode) {
        ringCountPipeline.setAnalysisRectMode(analysisRectMode);
    }

    // Save image //
    public void saveOutput(String filename) {
        ringCountPipeline.saveLatestMat(filename);
    }
    public void saveOutput() {
        RingCountPipeline.Viewport lastViewport = getViewport();
        setViewport(RingCountPipeline.Viewport.RAW_IMAGE); // TODO: test
//        try {
//            Thread.sleep(100);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        String filename = "/samples/" + "IMG_" + System.currentTimeMillis();
        saveOutput(filename);
        setViewport(lastViewport);
    }
}