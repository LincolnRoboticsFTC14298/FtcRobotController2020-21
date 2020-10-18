package org.firstinspires.ftc.teamcode.compenents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.Component;
import org.firstinspires.ftc.teamcode.util.RobotMap;

public class Drive implements Component {
    // Mecanum drive //
    Robot robot;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    public Drive(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        // Initialize motors //
        frontLeft = robot.hardwareMap.get(DcMotor.class, RobotMap.frontLeft);
        frontRight = robot.hardwareMap.get(DcMotor.class, RobotMap.frontRight);
        backLeft = robot.hardwareMap.get(DcMotor.class, RobotMap.backLeft);
        backRight = robot.hardwareMap.get(DcMotor.class, RobotMap.backRight);

        // Reverse direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        setPower(0,0,0,0);
        updateMotorPowers();
    }

    public void periodic() {
        teleop();
        updateMotorPowers();
    }

    public void end() {
        setPower(0,0,0,0);
        updateMotorPowers();
    }

    public void teleop() {
        double r = robot.oi.getLeftStickRadius();
        double angle = robot.oi.getLeftStickAngle() - Math.PI / 4;
        double xRot = robot.oi.getRightX();
        double fl = r * Math.cos(angle) + xRot;
        double fr = r * Math.sin(angle) - xRot;
        double bl = r * Math.sin(angle) + xRot;
        double br = r * Math.cos(angle) - xRot;
        setPower(fl, fr, bl, br);
    }

    public void setPower(double fl, double fr, double bl, double br) {
        // TODO: Clip powers to make sure it's between -1 and 1
        frontLeftPower = fl;
        frontRightPower = fr;
        backLeftPower = bl;
        backRightPower = br;
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

    public void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}
