package org.firstinspires.ftc.teamcode.compenents;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.Component;
import org.firstinspires.ftc.teamcode.util.RobotMap;

public class Arm implements Component {
    Robot robot;
    Servo armServo, clawServo;
    double armAngle, clawAngle;

    public Arm(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        armServo = robot.hardwareMap.get(Servo.class, RobotMap.ARM_SERVO_NAME);
        clawServo = robot.hardwareMap.get(Servo.class, RobotMap.CLAW_SERVO_NAME);
        closeClaw(); // At the beginning of the round, the claw is closed with the wobble
        setArmAngle(0);
        updateArmAngle();
    }

    @Override
    public void teleopPeriodic() {
        if (robot.oi.openClaw()) {
            openClaw();
        }
        if (robot.oi.closeClaw()) {
            closeClaw();
        }
        updateArmAngle();
    }

    @Override
    public void end() {
        closeClaw();
        updateArmAngle();
    }

    public void setArmAngle(double angle) {
        this.armAngle = angle;
    }

    public void setClawServo(double angle) {
        this.clawAngle = angle;
    }

    public void openClaw() {
        clawAngle = RobotMap.CLAW_OPEN_ANGLE;
    }

    public void closeClaw() {
        clawAngle = RobotMap.CLAW_CLOSE_ANGLE;
    }

    public void updateArmAngle() {
        armServo.setPosition(armAngle);
        clawServo.setPosition(clawAngle);
        robot.telemetry.addData("Arm servo positions: ", "%.5f arm, %.5f claw",
                armServo.getPosition(), clawServo.getPosition());
    }
}
