package org.firstinspires.ftc.teamcode.tuning;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline.Viewport;

import java.io.File;

@TeleOp(name="Launcher", group="Tuner")
public class CameraTuner extends OpMode {
    private FtcDashboard dashboard;
    private Robot robot = new Robot();
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        robot.init(this);
        robot.vision.startStreaming();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
            robot.vision.setViewport(Viewport.RAW_MASK);
        } else if (gamepad1.dpad_left) {
            robot.vision.setViewport(Viewport.MASKED);
        } else if (gamepad1.dpad_up) {
            robot.vision.setViewport(Viewport.DIST1);
        } else if (gamepad1.dpad_right) {
            robot.vision.setViewport(Viewport.DIST2);
        } else if (gamepad1.a) {
            robot.vision.setViewport(Viewport.MARKERS);
        } else if (gamepad1.b) {
            robot.vision.setViewport(Viewport.RAW_IMAGE);
        }

        if (gamepad1.x) {
            Viewport lastViewport = robot.vision.getViewport();
            File f = Environment.getDataDirectory()
                    .getAbsoluteFile()
                    .getParentFile()
                    .getParentFile();
            String path = f.getPath() + "/vision/samples/";
            String filename = path + "IMG_" + System.currentTimeMillis();
            robot.vision.saveOutput(filename);
            robot.vision.setViewport(lastViewport);
        }

        dashboard.sendImage(robot.vision.getOutput());
    }
}
