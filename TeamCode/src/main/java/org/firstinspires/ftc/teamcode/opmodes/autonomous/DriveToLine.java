package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.DataWriterUtil;
import org.firstinspires.ftc.teamcode.util.Field;

import static org.firstinspires.ftc.teamcode.util.Field.LAUNCH_LINE_X;

@Config
@Autonomous(name="Drive to line", group="Autonomous")
@Disabled
public class DriveToLine extends OpMode {
    public static Pose2d startPose = new Pose2d(0,0,0);

    Robot robot;

    @Override
    public void init() {
        DataWriterUtil.setAlliance(Field.Alliance.RED);
        robot = new Robot(this);
        robot.start();
    }

    @Override
    public void start() {
        robot.drive.setPoseEstimate(startPose);

        Pose2d pose = robot.localizer.getPoseEstimate();
        robot.drive.strafeToPoint(new Pose2d(LAUNCH_LINE_X, pose.getY(), 0));

        DataWriterUtil.setLastPose(robot.drive.getPoseEstimate());
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        robot.stop();
    }
}
