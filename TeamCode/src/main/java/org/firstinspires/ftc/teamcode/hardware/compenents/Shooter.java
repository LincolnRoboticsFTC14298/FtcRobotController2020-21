package org.firstinspires.ftc.teamcode.hardware.compenents;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.Field.Target;
import org.firstinspires.ftc.teamcode.hardware.RobotMap;

import com.qualcomm.robotcore.util.Range;

public class Shooter implements Subsystem {
    // Mecanum drive //
    private Robot robot;

    private DcMotor motor1;
    private DcMotor motor2;
    private Servo flap;

    private double motor1Power;
    private double motor2Power;
    private double flapAngle;
    private double targetAngle = -1;

    private Target target = Target.HIGH_GOAL;

    public Shooter(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        // Initialize motors //
        motor1 = robot.hardwareMap.get(DcMotor.class, RobotMap.MOTOR1_NAME);
        motor2 = robot.hardwareMap.get(DcMotor.class, RobotMap.MOTOR2_NAME);

        // Reverse direction
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        setPower(0);
        setFlapAngle(Math.toRadians(45));
        updateMotorPowers();
        updateServoAngle();
    }

    @Override
    public void update() {
        updateMotorPowers();
        updateServoAngle();
        robot.telemetry.addData("Target Angle: ",  Math.toDegrees(targetAngle));
        robot.telemetry.addData("Flap Angle: ",
                Math.toDegrees(posToAngle(flap.getPosition())));
    }

    @Override
    public void end() {
        setPower(0);
        setFlapAngle(Math.toRadians(45));
        updateMotorPowers();
        updateServoAngle();
    }

    public boolean ready() {
        if (targetAngle != -1) {
            return posToAngle(flap.getPosition()) - targetAngle < RobotMap.FLAP_MIN_ERROR;
        } else {
            return false;
        }
    }

    public void setTarget(Target target) {
        this.target = target;
    }

    public void aim() {
        Pose2d point = new Pose2d(target.location.x, target.location.y);
        Pose2d pos = robot.positionLocalizer.getPose();
        double dy = point.getY() - pos.getY();
        double dx = point.getX() - pos.getX();
        double dist = Math.hypot(dy, dx);
        double height = target.location.z;

        // Math stuff
        double angle = 0;
        setFlapAngle(targetAngle);
        targetAngle = angle;
    }

    public void setFlapAngle(double angle) {
        // Convert from 0 to 1
        // Angle is in radians
        flapAngle = angle;
    }

    public void setPower(double power) {
        // TODO: Clip powers to make sure it's between -1 and 1
        motor1Power = motor2Power = power;
    }

    private void updateMotorPowers() {
        motor1.setPower(motor1Power);
        motor2.setPower(motor2Power);
        robot.telemetry.addData("Shooter power: ",
                "%.6f", motor1.getPower());
    }

    private void updateServoAngle() {
        flap.setPosition(angleToPos(flapAngle));
    }

    private double posToAngle(double pos) {
        // Scale from 0 = minAngle to 1 = maxAngle
        return Range.scale(pos, 0,1, RobotMap.FLAP_MIN_ANGLE, RobotMap.FLAP_MAX_ANGLE);
    }

    private double angleToPos(double angle) {
        // Scale from minAngle = 0 to maxAngle = 1
        return Range.scale(angle, RobotMap.FLAP_MIN_ANGLE, RobotMap.FLAP_MAX_ANGLE, 0, 1);
    }
}
