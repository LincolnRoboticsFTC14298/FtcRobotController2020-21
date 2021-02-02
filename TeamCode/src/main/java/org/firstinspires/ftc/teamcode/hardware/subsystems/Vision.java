package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.graphics.Bitmap;
import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotlib.hardware.Subsystem;
import org.firstinspires.ftc.teamcode.util.Field;
import org.firstinspires.ftc.teamcode.util.Ring;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline;
import org.firstinspires.ftc.teamcode.vision.RingData;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.io.File;
import java.util.List;

import static org.firstinspires.ftc.robotlib.util.MathUtil.poseToVector2D;
import static org.firstinspires.ftc.robotlib.util.MathUtil.rotateVector;
import static org.firstinspires.ftc.robotlib.util.MathUtil.vector3DToVector2D;
import static org.firstinspires.ftc.robotlib.util.MathUtil.vectorFromAngle;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.CAMERA_LOCATION;
import static org.firstinspires.ftc.teamcode.util.Field.RING_DIAMETER;
import static org.opencv.android.Utils.matToBitmap;

public class Vision extends Subsystem {
    public static final int WIDTH = 320;
    public static final int HEIGHT = 240;
    public static final double FOV = 27.3; // degrees
    public static final double FUDGE_FACTOR = 1;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    //private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private OpenCvInternalCamera2 camera;
    private RingCountPipeline ringCountPipeline;

    private List<RingData> ringData;

    // TODO: Possibly delete later
    private boolean processed = false;
    private boolean viewportOn = false;

    Localizer localizer;

    public Vision(HardwareMap hardwareMap) {
        super("Vision");
        this.localizer = null;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        ringCountPipeline = new RingCountPipeline(camera);
        camera.setPipeline(ringCountPipeline);
        camera.openCameraDeviceAsync(
                () -> {
                    camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);
                    camera.setSensorFps(30);
                }
        );
    }

    public Vision(HardwareMap hardwareMap, Localizer localizer) {
        super("Vision");
        this.localizer = localizer;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        ringCountPipeline = new RingCountPipeline(camera);
        camera.setPipeline(ringCountPipeline);

        camera.openCameraDeviceAsync(
                () -> {
                    camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);
                    camera.setSensorFps(30);
                }
        );
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
        telemetry.put("Test", ringData == null);
        dashboard.sendImage(getOutput());
    }

    public void analyze() {
        ringData = ringCountPipeline.getRingData();
        processed = true;
    }

    public void resumeViewport() {
        camera.resumeViewport();
    }
    public void pauseViewport() {
        camera.pauseViewport();
    }


    public int getNumRings() {
        TelemetryPacket packet = new TelemetryPacket();
        if (processed) {
            return ringData.size();
        } else {
            packet.addLine("ERROR: IMAGE NOT PROCESSED");
            dashboard.sendTelemetryPacket(packet);
            return 0;
        }
    }

    public List<RingData> getRingData() {
        return ringData;
    }

    public double getRingLocalAngle(RingData ring) {
        // Assumes a centered camera
        // Alternative: angle = atan( atan(fov/2) * (2cx / w - 1) ))
        double cx = ring.getCentroid().x;
        int w = WIDTH;
        double x = cx - w/2; // centered at w/2
        return -Math.atan(Math.tan( Math.toRadians(FOV) / 2.0) * 2.0 * x / w);
    }
    public double getRingLocalDistance(RingData ring) {
        // inches
        double cx = ring.getCentroid().x;
        int w = WIDTH;
        double xpx = cx - w/2;
        double angle = getRingLocalAngle(ring);
        double dpx = ring.getBoxSize().width;
        return FUDGE_FACTOR * Math.abs(xpx / Math.sin(angle)) * RING_DIAMETER / dpx;
    }
    public Vector2D getRingLocalPosition(RingData ring) {
        // in frame of rotated robot
        double angle = getRingLocalAngle(ring);
        double distance = getRingLocalDistance(ring);
        return vector3DToVector2D(CAMERA_LOCATION)
                .add(vectorFromAngle(distance, angle));
    }
    public Vector2D getRingPosition(RingData ringData) {
        Pose2d pose = localizer.getPoseEstimate();
        return rotateVector(getRingLocalPosition(ringData), pose.getHeading())
                .add(poseToVector2D(pose));
    }
    public void scan() {
        analyze();
        for (RingData r : ringData) {
            Ring ring = new Ring(getRingPosition(r));
            Field.addRing(ring);
        }
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
    public void setCroppedRectMode(RingCountPipeline.CroppedRectMode croppedRectMode) {
        ringCountPipeline.setCroppedRectMode(croppedRectMode);
    }

    public Bitmap getOutput() {
        Mat mat = ringCountPipeline.getLatestMat();
        Bitmap bmt = Bitmap.createBitmap(mat.cols(), mat.rows(),
                Bitmap.Config.RGB_565);
        matToBitmap(mat, bmt);
        return bmt;
    }
    public void saveOutput(String filename) {
        ringCountPipeline.saveLatestMat(filename);
    }
    public void saveOutput() {
        RingCountPipeline.Viewport lastViewport = getViewport();
        File f = Environment.getDataDirectory()
                .getAbsoluteFile()
                .getParentFile()
                .getParentFile()
                .getParentFile();
        String path = f.getPath() + "/vision/samples/";
        String filename = path + "IMG_" + System.currentTimeMillis();
        saveOutput(filename);
        setViewport(lastViewport);
    }
}
