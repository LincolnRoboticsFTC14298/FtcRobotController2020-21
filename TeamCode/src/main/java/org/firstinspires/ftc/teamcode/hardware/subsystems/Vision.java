package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;
import org.firstinspires.ftc.robotlib.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Field;
import org.firstinspires.ftc.teamcode.util.Ring;
import org.firstinspires.ftc.teamcode.vision.RingData;
import org.firstinspires.ftc.teamcode.vision.RingPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.CAMERA_LOCATION;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.CAMERA_PITCH;
import static org.firstinspires.ftc.teamcode.util.Ring.RING_DIAMETER;

public class Vision extends AbstractSubsystem {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    public static int WIDTH = 320;
    public static int HEIGHT = 240;
    public static final double FOV_X = Math.toRadians(27.3), FOV_Y = Math.toRadians(21); // radians
    public static double FUDGE_FACTOR_Y = 1, FUDGE_FACTOR_X = 1;

    public static double RING_AREA_MIN = 0.0001;

    public static int oneRingHeight = 15;
    public static int zeroRingHeight = 2;

    private OpenCvInternalCamera2 camera;
    private RingPipeline ringPipeline;

    private List<RingData> ringData;

    private Localizer localizer;

    public Vision(HardwareMap hardwareMap, Localizer localizer) {
        super("Vision");
        this.localizer = localizer;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        ringPipeline = new RingPipeline();
        camera.setPipeline(ringPipeline);

        camera.openCameraDeviceAsync(
                () -> {
                    camera.setViewportRenderingPolicy(OpenCvInternalCamera2.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                    camera.setViewportRenderer(OpenCvInternalCamera2.ViewportRenderer.GPU_ACCELERATED);
                    camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
                    setFPS(30);
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
        ringPipeline.updateTelemetry();
        if (ringData != null) telemetry.put("Number of Rings", getCenterStackSize());
        telemetry.put("Viewport", getViewport());
        telemetry.putAll(ringPipeline.getTelemetryData());
    }

    @Override
    public void updateLogging() {
        ringPipeline.updateLogging();
    }

    public void analyze() {
        ringData = ringPipeline.getRingData();
    }
    public void scan() {
        analyze();
        for (RingData r : ringData) {
            Ring ring = new Ring(getRingPosition(r));
            Field.ringProvider.add(ring);
        }
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

    /*
     * Frame of reference: robot at center, axis is same as robots
     */
    public Vector3D getRingLocalPosition(RingData ring) {
        Vector3D cam = getRingCameraLocalPosition(ring);
        Vector3D rotCam = MathUtil.rotateY(cam, CAMERA_PITCH);
        return CAMERA_LOCATION.add(rotCam);
    }

    /*
     * Frame of reference: global
     */
    public Vector2d getRingPosition(RingData ring) {
        Pose2d pose = localizer.getPoseEstimate();
        Vector3D ringLocal3D = getRingLocalPosition(ring);
        Vector2d ringLocal2d = MathUtil.vector3DToVector2d(ringLocal3D);
        return MathUtil.localToGlobal(ringLocal2d, pose);
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

    public void setFPS(int fps) {
        FtcDashboard.getInstance().stopCameraStream();
        camera.setSensorFps(fps);
        FtcDashboard.getInstance().startCameraStream(camera, fps);
    }

    public int getCenterStackSize() {
        analyze();
        if (ringData.size() > 0) {
            RingData centerRing = ringData.get(0);
            if (centerRing.getNormalizedContourArea() >= RING_AREA_MIN) {
                double height = centerRing.getBoxSize().height;
                telemetry.put("Height", height);
                if (height > oneRingHeight) {
                    return 4;
                } else if (height > zeroRingHeight) {
                    return 1;
                }
            }
        }
        return 0;
    }

    public RingPipeline.Viewport getViewport() {
        return ringPipeline.getViewport();
    }
    public void setViewport(RingPipeline.Viewport viewport) {
        ringPipeline.setViewport(viewport);
    }

    public void setClose(boolean close) {
        ringPipeline.setClose(close);
    }
    public void setCroppedRectMode(RingPipeline.AnalysisRectMode analysisRectMode) {
        ringPipeline.setAnalysisRectMode(analysisRectMode);
    }

    // Save image //
    public void saveOutput(String filename) {
        ringPipeline.saveLatestMat(filename);
    }
    public void saveOutput() {
        RingPipeline.Viewport lastViewport = getViewport();
        setViewport(RingPipeline.Viewport.RAW_IMAGE); // test
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
