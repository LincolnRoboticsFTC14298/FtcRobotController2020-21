package org.firstinspires.ftc.teamcode.hardware.compenents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.Field.Target;
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
        updateMotorPowers();
    }

    @Override
    public void end() {
        setPower(0,0,0,0);
        updateMotorPowers();
    }

    public void teleopControl(double radius, double angle, double rotation, boolean localControl, boolean autoAim) {
        // In frame of refrence of the robot
        angle -= Math.PI / 4; // Strafing angle
        if (localControl) {
            angle -= Math.PI / 2 +  Math.toRadians(robot.positionLocalizer.getHeading());
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

    public void setTarget(Target target) {
        this.target = target;
    }

    private double getAutoAimRotation() {
        return 0;
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
}
