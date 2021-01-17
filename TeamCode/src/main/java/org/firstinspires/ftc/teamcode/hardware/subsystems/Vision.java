package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.graphics.Bitmap;
import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.robotlib.hardware.Subsystem;
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

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.CAMERA_LOCATION;
import static org.firstinspires.ftc.teamcode.robotlib.util.MathUtil.poseToVector2D;
import static org.firstinspires.ftc.teamcode.robotlib.util.MathUtil.rotateVector;
import static org.firstinspires.ftc.teamcode.robotlib.util.MathUtil.vector3DToVector2D;
import static org.firstinspires.ftc.teamcode.robotlib.util.MathUtil.vectorFromAngle;
import static org.firstinspires.ftc.teamcode.util.Field.RING_DIAMETER;
import static org.opencv.android.Utils.matToBitmap;

public class Vision extends Subsystem {
    public static final int WIDTH = 320;
    public static final int HEIGHT = 240;
    public static final double FOV = 27.3; // degrees
    public static final double FUDGE_FACTOR = 1;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    //private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private OpenCvInternalCamera2 phoneCam;
    private RingCountPipeline ringCountPipeline;

    private List<RingData> ringData;

    // TODO: Possibly delete later
    private boolean processed = false;
    private boolean streaming = false;

    Localizer localizer;

    public Vision(HardwareMap hardwareMap) {
        super("Vision");
        this.localizer = null;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDeviceAsync(
                () -> {
                    ringCountPipeline = new RingCountPipeline(phoneCam);
                    phoneCam.setPipeline(ringCountPipeline);
                    phoneCam.setSensorFps(30);
                }
        );
    }

    public Vision(HardwareMap hardwareMap, Localizer localizer) {
        super("Vision");
        this.localizer = localizer;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDeviceAsync(
                () -> {
                    ringCountPipeline = new RingCountPipeline(phoneCam);
                    phoneCam.setPipeline(ringCountPipeline);
                    phoneCam.setSensorFps(30);
                }
        );
    }


    @Override
    public void update() {

    }

    @Override
    public void stop() {
        stopStreaming();
    }

    public void analyze() {
        startStreaming();
        ringData = ringCountPipeline.getRings();

        processed = true;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Number of Rings: ", ringData);
        dashboard.sendTelemetryPacket(packet);
        stopStreaming();
    }

    public void startStreaming() {
        if (!streaming) {
            phoneCam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);
            streaming = true;
        }
    }
    public void stopStreaming() {
        if (streaming) {
            streaming = false;
            phoneCam.stopStreaming();
        }
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
        double cx = ring.centroid.x;
        int w = WIDTH;
        double x = cx - w/2; // centered at w/2
        return -Math.atan(Math.tan( Math.toRadians(FOV) / 2.0) * 2.0 * x / w);
    }
    public double getRingLocalDistance(RingData ring) {
        // inches
        double cx = ring.centroid.x;
        int w = WIDTH;
        double xpx = cx - w/2;
        double angle = getRingLocalAngle(ring);
        double dpx = ring.boxSize.width;
        return FUDGE_FACTOR * Math.abs(xpx / Math.sin(angle)) * RING_DIAMETER / dpx;
    }
    public Vector2D getRingLocalPosition(RingData ring) {
        // in frame of robot
        double angle = getRingLocalAngle(ring);
        double distance = getRingLocalDistance(ring);
        return vector3DToVector2D(CAMERA_LOCATION)
                .add(vectorFromAngle(angle, distance));
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

    public Bitmap getOutput() {
        Mat mat = ringCountPipeline.getLatestMat();
        Bitmap bmt = null;
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
