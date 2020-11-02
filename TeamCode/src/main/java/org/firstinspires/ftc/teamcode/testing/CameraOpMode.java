package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.RingCountPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@TeleOp()
public class CameraOpMode extends LinearOpMode {

    OpenCvInternalCamera2 phoneCam;
    RingCountPipeline ringCountPipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        ringCountPipeline = new RingCountPipeline(phoneCam, RingCountPipeline.Viewport.ANNOTATED);
        phoneCam.setPipeline(ringCountPipeline);

        /*
         * Start streaming
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Demonstrate how to turn on the flashlight
         */
        //phoneCam.setFlashlightEnabled(true);

        /*
         * Demonstrate how to lock the camera hardware to sending frames at 30FPS
         */
        //phoneCam.setSensorFps(30);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Number of Rings", ringCountPipeline.getNumRingsFound());
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();
            sleep(100);
        }
    }
}