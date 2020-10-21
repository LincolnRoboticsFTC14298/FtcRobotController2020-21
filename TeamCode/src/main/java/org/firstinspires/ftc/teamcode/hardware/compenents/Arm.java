package org.firstinspires.ftc.teamcode.hardware.compenents;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.RobotMap;

public class Arm implements Subsystem {
    private Robot robot;

    private Servo armServo , clawServo;
    private double armPosition, clawPosition;

    public Arm(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        clawServo = robot.hardwareMap.get(Servo.class, RobotMap.CLAW_SERVO_NAME);
        armServo = robot.hardwareMap.get(Servo.class, RobotMap.ARM_SERVO_NAME);

        closeClaw(); // At the beginning of the round, the claw is closed with the wobble
        defaultArm();
        updateServoPositions();
    }

    @Override
    public void update() {
        updateServoPositions();
    }

    @Override
    public void end() {
        closeClaw();
        defaultArm();
        updateServoPositions();
    }

    public void setArmAngle(double position) {
        this.armPosition = position;
    }

    public void setClawServo(double position) {
        this.clawPosition = position;
    }

    public void openClaw() {
        setClawServo(RobotMap.CLAW_OPEN_POSITION);
    }
    public void closeClaw() {
        setClawServo(RobotMap.CLAW_CLOSE_POSITION);
    }

    public void liftArm() {
        setArmAngle(RobotMap.ARM_LIFT_POSITION);
    }
    public void defaultArm() {
        setArmAngle(RobotMap.ARM_DEFAULT_POSITION);
    }
    public void lowerArm() {
        setArmAngle(RobotMap.ARM_LOWER_POSITION);
    }

    public void updateServoPositions() {
        armServo.setPosition(armPosition);
        clawServo.setPosition(clawPosition);
        robot.telemetry.addData("Arm servo positions: ", "%.5f arm, %.5f claw",
                armServo.getPosition(), clawServo.getPosition());
    }
}
