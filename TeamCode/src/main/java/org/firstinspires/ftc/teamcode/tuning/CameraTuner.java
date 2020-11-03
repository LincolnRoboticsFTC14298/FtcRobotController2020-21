package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline.Viewport;

@TeleOp(name="Camera", group="Tuner")
public class CameraTuner extends OpMode {
    private FtcDashboard dashboard;
    private Robot robot = new Robot();
    private RadicalGamepad gamepad;
    
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        robot.init(this);
        robot.vision.startStreaming();
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.update();
        if (gamepad.dpad_down) {
            robot.vision.setViewport(Viewport.RAW_MASK);
        } else if (gamepad.dpad_left) {
            robot.vision.setViewport(Viewport.MASKED);
        } else if (gamepad.dpad_up) {
            robot.vision.setViewport(Viewport.DIST1);
        } else if (gamepad.dpad_right) {
            robot.vision.setViewport(Viewport.DIST2);
        } else if (gamepad.a) {
            robot.vision.setViewport(Viewport.MARKERS);
        } else if (gamepad.b) {
            robot.vision.setViewport(Viewport.RAW_IMAGE);
        }

        if (gamepad.x) {
            robot.vision.saveOutput();
        }

        dashboard.sendImage(robot.vision.getOutput());
    }
}
