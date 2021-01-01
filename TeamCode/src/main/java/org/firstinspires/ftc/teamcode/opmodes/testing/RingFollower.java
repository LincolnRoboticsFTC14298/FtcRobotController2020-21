package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Ring Follower Test", group="Test")
public class RingFollower extends OpMode {
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
        if (robot.vision.getNumRings() != 0) {
            double angle = robot.vision.getRingAngle(robot.vision.getRings().get(0));
            if (angle > 0.1) {
                robot.drive.turnAsync(-angle);
            } else {
                double dist = robot.vision.getRingDistance(robot.vision.getRings().get(0)) + 5;
                // aligned, drive forward distance + a bit
                Trajectory traj = robot.drive.trajectoryBuilder(new Pose2d()).forward(dist).build();
                robot.drive.followTrajectory(traj);
            }
        }

        robot.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
