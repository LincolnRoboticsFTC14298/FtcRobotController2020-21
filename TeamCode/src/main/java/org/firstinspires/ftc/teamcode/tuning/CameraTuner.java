package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline.Viewport;

@TeleOp()
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
        }
        dashboard.sendImage(robot.vision.getOutput());
    }
}
