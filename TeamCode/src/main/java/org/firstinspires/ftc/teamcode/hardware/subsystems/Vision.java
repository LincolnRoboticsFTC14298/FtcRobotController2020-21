package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotlib.hardware.Subsystem;
import org.firstinspires.ftc.teamcode.util.Field;
import org.firstinspires.ftc.teamcode.util.Ring;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline;
import org.firstinspires.ftc.teamcode.vision.RingData;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.io.File;
import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.CAMERA_LOCATION_2d;
import static org.firstinspires.ftc.teamcode.util.Ring.RING_DIAMETER;

public class Vision extends Subsystem {
    public static int WIDTH = 320;
    public static int HEIGHT = 240;
    public static final double FOV = 27.3; // degrees
    public static double FUDGE_FACTOR = 1;

    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private OpenCvInternalCamera2 camera;
    private RingCountPipeline ringCountPipeline;

    private List<RingData> ringData;

    // TODO: Possibly delete later
    private boolean processed = false;

    private Localizer localizer;

    public Vision(HardwareMap hardwareMap, Localizer localizer) {
        super("Vision");
        this.localizer = localizer;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        ringCountPipeline = new RingCountPipeline();
        camera.setPipeline(ringCountPipeline);

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
        if (ringData != null) telemetry.put("Number of Rings", ringData.size());
        telemetry.put("Output type", ringCountPipeline.getLatestMat().type());
        telemetry.put("Viewport", getViewport());
    }

    public void analyze() {
        ringData = ringCountPipeline.getRingData();
        processed = true;
    }
    public void scan() {
        analyze();
        for (RingData r : ringData) {
            Ring ring = new Ring(getRingPosition(r));
            Field.ringProvider.add(ring);
        }
    }

    /*
     * Frame of reference: camera center and axis is local
     */
    public double getCameraRingAngle(RingData ring) {
        double cx = ring.getCentroid().x;
        int w = WIDTH;
        double x = cx - w/2; // centered at w/2
        return -Math.atan(Math.tan( Math.toRadians(FOV) / 2.0) * 2.0 * x / w);
    }
    public double getCameraRingDistance(RingData ring) {
        double cx = ring.getCentroid().x;
        int w = WIDTH;
        double xpx = cx - w/2;
        double angle = getCameraRingAngle(ring);
        double dpx = ring.getBoxSize().width;
        return FUDGE_FACTOR * Math.abs(xpx / Math.sin(angle)) * RING_DIAMETER / dpx;
    }

    /*
     * Frame of reference: robot at center and axis is local
     */
    public Vector2d getRingLocalPosition(RingData ring) {
        double angle = getCameraRingAngle(ring);
        double distance = getCameraRingDistance(ring);
        return CAMERA_LOCATION_2d.plus(Vector2d.polar(distance, angle));
    }

    /*
     * Frame of reference: global
     */
    public Vector2d getRingPosition(RingData ringData) {
        Pose2d pose = localizer.getPoseEstimate();
        return getRingLocalPosition(ringData).rotated(pose.getHeading())
                .plus(pose.vec());
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
        TelemetryPacket packet = new TelemetryPacket();
        if (processed) {
            return ringData.size();
        } else {
            packet.addLine("ERROR: IMAGE NOT PROCESSED");
            //dashboard.sendTelemetryPacket(packet);
            return 0;
        }
    }

    public void setFPS(int fps) {
        FtcDashboard.getInstance().stopCameraStream();
        camera.setSensorFps(fps);
        FtcDashboard.getInstance().startCameraStream(camera, fps);
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
        setViewport(RingCountPipeline.Viewport.RAW_IMAGE);
        File file = new File(Environment.getExternalStorageDirectory() + "/vision/samples/");
        String filename = file.getPath() + "IMG_" + System.currentTimeMillis();
        saveOutput(filename);
        setViewport(lastViewport);
    }
}
