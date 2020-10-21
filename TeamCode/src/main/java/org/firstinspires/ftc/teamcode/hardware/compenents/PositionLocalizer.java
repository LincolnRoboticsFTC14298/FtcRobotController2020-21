package org.firstinspires.ftc.teamcode.hardware.compenents;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;

public class PositionLocalizer implements Subsystem {
    Robot robot;

    private Pose2d pose;

    public PositionLocalizer(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public void end() {

    }

    public Pose2d getPose() {
        return pose;
    }

    public double getHeading() {
        return pose.getHeading();
    }
}
