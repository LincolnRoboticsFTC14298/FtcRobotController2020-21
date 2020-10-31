package org.firstinspires.ftc.teamcode.vision.old;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingCountPipeline extends OpenCvPipeline {
    OpenCvInternalCamera2 cam;
    boolean viewportPaused = false;
    double AREA_THRESHOLD = 700;
    int THICKNESS = 4;
    int rings = 0;

    public RingCountPipeline(OpenCvInternalCamera2 cam) {
        this.cam = cam;
    }

    @Override
    public Mat processFrame(Mat input) {
        rings = CameraRingDetection.ringCountUsingSegmentation(input, AREA_THRESHOLD,
                THICKNESS, false);
        return input;
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

    public int getNumRingsFound() {
        return rings;
    }
}
