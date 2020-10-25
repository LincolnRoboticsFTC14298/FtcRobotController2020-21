package org.firstinspires.ftc.teamcode.hardware.subsystems;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

public class Vision implements Subsystem {
    private Robot robot;

    OpenCvInternalCamera2 phoneCam;
    RingCountPipeline ringCountPipeline;

    private int rings = -1;

    // Can delete later
    private boolean processed = false;

    public Vision(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        this.robot = robot;
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        ringCountPipeline = new RingCountPipeline(phoneCam,true);
        phoneCam.setPipeline(ringCountPipeline);

        phoneCam.setSensorFps(30);
    }

    @Override
    public void update() {

    }

    @Override
    public void end() {

    }

    public void analyze() {
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        rings = ringCountPipeline.getNumRingsFound();
        phoneCam.stopStreaming();
        processed = true;
        robot.telemetry.addData("Number of Rings: ", rings);
    }

    public int getNumRings() {
        if (processed) {
            return rings;
        } else {
            robot.telemetry.addData("ERROR: ", "IMAGE NOT PROCESSED");
            return -1;
        }
    }
}
