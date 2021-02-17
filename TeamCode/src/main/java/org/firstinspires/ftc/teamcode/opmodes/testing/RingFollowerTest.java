package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Ring Follower Test", group="Test")
@Disabled
public class RingFollowerTest extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.vision.scan();

        goToRing();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public void goToRing() {
        robot.drive.goToRing();
        robot.drive.strafeToPoint(new Pose2d(0,0));
    }
}
