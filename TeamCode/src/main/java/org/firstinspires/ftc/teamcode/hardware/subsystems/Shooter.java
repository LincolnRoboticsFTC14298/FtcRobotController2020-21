package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.Field.*;
import org.firstinspires.ftc.teamcode.hardware.RobotMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Shooter implements Subsystem {
    // Mecanum drive //
    private Robot robot;

    private DcMotor motor1, motor2, loadMotor;
    private Servo flap;

    private double motor1Power, motor2Power, loadMotorPower;
    private double flapAngle;
    private double targetAngle = -1;

    private Target target = Target.HIGH_GOAL;
    private Alliance alliance = Alliance.BLUE;

    private double shootScheduler = 0;
    private boolean loading = false;

    public Shooter(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        // Initialize motors and servos //
        motor1 = robot.hardwareMap.get(DcMotor.class, RobotMap.MOTOR1_NAME);
        motor2 = robot.hardwareMap.get(DcMotor.class, RobotMap.MOTOR2_NAME);
        loadMotor = robot.hardwareMap.get(DcMotor.class, RobotMap.LOAD_MOTOR_NAME);

        flap = robot.hardwareMap.get(Servo.class, RobotMap.FLAP_NAME);

        // Reverse direction
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        setShooterPower(0);
        setLoadPower(0);
        setFlapAngle(Math.toRadians(45));
        updateMotorPowers();
        updateServoPositions();
    }

    @Override
    public void update() {
        setShooterPower(RobotMap.SHOOTER_DEFAULT_POWER);

        if (!doneShooting()) {
            aim();
            if (readyToShoot()) {
                startLoadingRing();
                shootScheduler -= 1;
            }
        }

        if (loading) {
            updateLoadingRing();
        }

        updateMotorPowers();
        updateServoPositions();
    }

    @Override
    public void end() {
        setShooterPower(0);
        setLoadPower(0);
        setFlapAngle(Math.toRadians(45));
        updateMotorPowers();
        updateServoPositions();
    }

    public void shoot() {
        shootScheduler += 1;
    }

    public void aim() {
        Pose2d point = target.getPose(robot.alliance);
        Pose2d pos = robot.positionLocalizer.getPose();
        double dy = point.getY() - pos.getY();
        double dx = point.getX() - pos.getX();
        double dist = Math.hypot(dy, dx);
        double height = target.getLocation(robot.alliance).getZ();

        // Math stuff
        double angle = 0;
        setFlapAngle(targetAngle);
        targetAngle = angle;
        setFlapAngle(targetAngle);
    }










    ElapsedTime loadingElapse = new ElapsedTime();
    public void startLoadingRing() {
        loading = true;
        loadingElapse.reset();
        setLoadPower(RobotMap.LOAD_MOTOR_POWER);
    }
    public void updateLoadingRing() {
        if (loadingElapse.milliseconds() / 1000.0 > RobotMap.LOAD_MOTOR_DELAY) {
            loading = false;
            setLoadPower(0);
        }
    }

    public boolean readyToShoot() {
        if (targetAngle != -1) {
            return posToAngle(flap.getPosition()) - targetAngle < RobotMap.FLAP_MIN_ERROR &&
                    motor1.getPower() - motor1Power < RobotMap.SHOOTER_MIN_ERROR &&
                    motor2.getPower() - motor2Power < RobotMap.SHOOTER_MIN_ERROR &&
                    !loading;
        } else {
            return false;
        }
    }
    public boolean doneShooting() {
        return shootScheduler == 0;
    }

    // Angle is in radians
    public void setFlapAngle(double angle) {
        flapAngle = angle;
    }
    public void setShooterPower(double power) {
        // TODO: Clip powers to make sure it's between -1 and 1
        double p = Range.clip(power, -1, 1);
        motor1Power = motor2Power = p;
    }
    public void setLoadPower(double power) {
        loadMotorPower = Range.clip(power, -1, 1);
    }

    private double posToAngle(double pos) {
        // Scale from 0 = minAngle to 1 = maxAngle
        return Range.scale(pos, 0,1, RobotMap.FLAP_MIN_ANGLE, RobotMap.FLAP_MAX_ANGLE);
    }
    private double angleToPos(double angle) {
        // Scale from minAngle = 0 to maxAngle = 1
        return Range.scale(angle, RobotMap.FLAP_MIN_ANGLE, RobotMap.FLAP_MAX_ANGLE, 0, 1);
    }

    private void updateMotorPowers() {
        motor1.setPower(motor1Power);
        motor2.setPower(motor2Power);
        loadMotor.setPower(loadMotorPower);
        robot.telemetry.addData("Shooter power: ",
                "%.5f", motor1.getPower());
    }
    private void updateServoPositions() {
        flap.setPosition(angleToPos(flapAngle));
        robot.telemetry.addData("Flap Angle: ",  "%.5f target %.5f actual",
                Math.toDegrees(targetAngle), Math.toDegrees(posToAngle(flap.getPosition())));

    }

    public void setTarget(Target target) {
        this.target = target;
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }
}
