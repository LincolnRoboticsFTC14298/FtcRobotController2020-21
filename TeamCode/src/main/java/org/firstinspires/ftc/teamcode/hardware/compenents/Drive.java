package org.firstinspires.ftc.teamcode.hardware.compenents;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.Field.*;
import org.firstinspires.ftc.teamcode.hardware.RobotMap;

public class Drive implements Subsystem {
    // Mecanum drive //
    private Robot robot;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

//    private boolean autoAim = false;
//    private boolean localControl = true;

    private Target target = Target.HIGH_GOAL;
    private Alliance alliance = Alliance.BLUE;

    public boolean aligning = false;

    public Drive(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        // Initialize motors //
        frontLeft = robot.hardwareMap.get(DcMotor.class, RobotMap.FRONT_LEFT_NAME);
        frontRight = robot.hardwareMap.get(DcMotor.class, RobotMap.FRONT_RIGHT_NAME);
        backLeft = robot.hardwareMap.get(DcMotor.class, RobotMap.BACK_LEFT_NAME);
        backRight = robot.hardwareMap.get(DcMotor.class, RobotMap.BACK_RIGHT_NAME);

        // Reverse direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        setPower(0,0,0,0);
        updateMotorPowers();
    }

    @Override
    public void update() {
        if (aligning && isAligned()) {
            aligning = false;
            setPower(0,0,0,0);
        }
        if (aligning) {
            pointAtTarget();
        }
        updateMotorPowers();
    }

    @Override
    public void end() {
        setPower(0,0,0,0);
        updateMotorPowers();
    }

    public void teleopControl(double radius, double angle, double rotation, boolean localControl, boolean autoAim) {
        angle -= Math.PI / 4; // Strafing angle
        if (localControl) {
            angle -= Math.PI / 2 +  Math.toRadians(getHeading());
        }
        if (autoAim) {
            rotation = getAutoAimRotation(); // Rotation amount for auto
        }

        double fl = radius * Math.cos(angle) + rotation;
        double fr = radius * Math.sin(angle) - rotation;
        double bl = radius * Math.sin(angle) + rotation;
        double br = radius * Math.cos(angle) - rotation;
        setPower(fl, fr, bl, br);
    }

    public boolean isAligned() {
        // In degrees
        return getHeading() - targetHeading() < RobotMap.DRIVER_TARGET_ANGLE_MIN_ERROR;
    }

    public void pointAtTarget() {
        aligning = true;
        double rotation = getAutoAimRotation();
        setPower(rotation, -rotation, rotation, -rotation);
    }

    private double targetHeading() {
        Pose2d point = target.getPose(robot.alliance);
        Pose2d pos = getPose();
        double dy = point.getY() - pos.getY() - RobotMap.SHOOTER_LOCATION.getY();
        double dx = point.getX() - pos.getX() - RobotMap.SHOOTER_LOCATION.getX(); // X is forward direction
        double targetHeading = Math.atan2(dy, dx); // Happens to be in the heading frame
        return Math.toDegrees(targetHeading);
    }

    private double getAutoAimRotation() {
        double diffHeading = Math.toRadians(getHeading() - targetHeading());
        // TODO: CHANGE TO PID OR ROADRUNNER OR SOMETHING
        return diffHeading*0.1;
    }

    private double getHeading() {
        // Returns in degrees
        return getPose().getHeading();
    }
    private Pose2d getPose() {
        return robot.positionLocalizer.getPose();
    }

    public void setPower(double fl, double fr, double bl, double br) {
        // TODO: Clip powers to make sure it's between -1 and 1
        frontLeftPower = fl;
        frontRightPower = fr;
        backLeftPower = bl;
        backRightPower = br;
    }
    public void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void updateMotorPowers() {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        robot.telemetry.addData("Motor powers: ",
                "%.6f fl   %.6f fr   %.6f bl   %.6f br",
                frontLeft.getPower(), frontRight.getPower(),
                backLeft.getPower(), backRight.getPower());
    }

    public void setTarget(Target target) {
        this.target = target;
    }
    public void setAlliance(Alliance alliance) {this.alliance = alliance;}
}
