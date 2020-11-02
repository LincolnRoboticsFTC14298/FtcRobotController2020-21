package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline.Viewport;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.Arrays;

import static org.opencv.android.Utils.matToBitmap;

public class Vision implements Subsystem {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet;

    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private OpenCvInternalCamera2 phoneCam;
    private RingCountPipeline ringCountPipeline;

    private int rings;

    // TODO: Possibly delete later
    private boolean processed = false;

    public Vision() {

    }

    @Override
    public void init(HardwareMap hardwareMap) {
        packet = new TelemetryPacket();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        ringCountPipeline = new RingCountPipeline(phoneCam, Viewport.ANNOTATED);
                        phoneCam.setPipeline(ringCountPipeline);
                        phoneCam.setSensorFps(30);
                    }
                }
        );

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void update() {
        packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        packet = new TelemetryPacket();
        stopStreaming();
        dashboard.sendTelemetryPacket(packet);
    }

    public void analyze() {
        rings = ringCountPipeline.getNumRingsFound();
        processed = true;
        packet.put("Number of Rings: ", rings);
    }

    public void startStreaming() {
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }
    public void stopStreaming() {
        phoneCam.stopStreaming();
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

    public int getNumRings() {
        if (processed && Arrays.asList(new int[]{0,1,4}).contains(rings)) {
            return rings;
        } else if (processed) {
            packet.addLine("Ring is not 0,1,4!");
            return 0;
        } else {
            packet.addLine("ERROR: IMAGE NOT PROCESSED");
            return 0;
        }
    }
}
