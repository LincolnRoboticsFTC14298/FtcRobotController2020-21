package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.graphics.Bitmap;
import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.RingCountPipeline;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline.Viewport;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.io.File;
import java.util.Arrays;

import robotlib.hardware.Subsystem;

import static org.opencv.android.Utils.matToBitmap;

public class Vision extends Subsystem {
    public static final int width = 320;
    public static final int height = 240;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    //private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private OpenCvInternalCamera2 phoneCam;
    private RingCountPipeline ringCountPipeline;

    private int rings;

    // TODO: Possibly delete later
    private boolean processed = false;

    public Vision(HardwareMap hardwareMap) {
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
        rings = ringCountPipeline.getNumRingsFound();
        processed = true;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Number of Rings: ", rings);
        dashboard.sendTelemetryPacket(packet);
    }

    public void startStreaming() {
        phoneCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
    }
    public void stopStreaming() {
        phoneCam.stopStreaming();
    }

    public Viewport getViewport() {
        return ringCountPipeline.getViewport();
    }
    public void setViewport(Viewport viewport) {
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
        Viewport lastViewport = getViewport();
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

    public int getNumRings() {
        TelemetryPacket packet = new TelemetryPacket();
        if (processed && Arrays.asList(new int[]{0,1,4}).contains(rings)) {
            return rings;
        } else if (processed) {
            packet.addLine("Ring is not 0,1,4!");
            dashboard.sendTelemetryPacket(packet);
            return 0;
        } else {
            packet.addLine("ERROR: IMAGE NOT PROCESSED");
            dashboard.sendTelemetryPacket(packet);
            return 0;
        }

    }
}
