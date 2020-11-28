package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.RadicalOpMode;
import org.firstinspires.ftc.teamcode.util.Field;

@Config
@Autonomous(name="Drive to line", group="Autonomous")
public class DriveToLine extends RadicalOpMode {
    public static double FORWARD_DISTANCE = 60;
    public static double STRAFE_DISTANCE = 10;
    Robot robot;

    @Override
    public void init() {
        setAlliance(Field.Alliance.RED);
        robot = new Robot(this);
        robot.init();
    }

    @Override
    public void start() {
        robot.vision.startStreaming();
        robot.vision.analyze();
        robot.vision.stopStreaming();


        Trajectory traj1 = robot.drive.trajectoryBuilder(new Pose2d())
                .strafeRight(STRAFE_DISTANCE)
                .build();

        robot.drive.followTrajectory(traj1);


        Trajectory traj2 = robot.drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD_DISTANCE)
                .build();

        robot.drive.followTrajectory(traj2);

        setLastPose(robot.drive.getPoseEstimate());
//
//        setPower(1);
//        try {
//            sleep(1000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        setPower(0);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        robot.stop();
    }

    public void setPower(double n) {
        robot.drive.setMotorPowers(n,n,n,n);
        robot.update();
    }
}
