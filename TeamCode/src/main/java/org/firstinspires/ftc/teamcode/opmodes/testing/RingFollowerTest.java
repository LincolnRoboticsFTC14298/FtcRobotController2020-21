package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Ring Follower Test", group="Test")
public class RingFollowerTest extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.init();
    }

    @Override
    public void start() {
        robot.start();
        robot.vision.startStreaming();
    }

    @Override
    public void loop() {
        robot.vision.scan();
        robot.goToRing();

        robot.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
