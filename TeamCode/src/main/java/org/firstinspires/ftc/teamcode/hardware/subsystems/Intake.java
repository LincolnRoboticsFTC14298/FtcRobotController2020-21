package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.RobotMap;

public class Intake implements Subsystem {
    private Robot robot;

    private DcMotor motor1, motor2;
    private boolean on = false;

    public Intake(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        motor1 = robot.hardwareMap.get(DcMotor.class, RobotMap.INTAKE_MOTOR1_NAME);
        motor2 = robot.hardwareMap.get(DcMotor.class, RobotMap.INTAKE_MOTOR2_NAME);

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        turnOff();
        updateIntakeMotors();
    }

    @Override
    public void update() {
        updateIntakeMotors();
    }

    @Override
    public void end() {
        turnOff();
        updateIntakeMotors();
    }

    public void turnOn() {
        on = true;
    }

    public void turnOff() {
        on = false;
    }

    public void updateIntakeMotors() {
        if (on) {
            motor1.setPower(RobotMap.INTAKE_MOTOR1_POWER_ON);
            motor2.setPower(RobotMap.INTAKE_MOTOR2_POWER_ON);
        } else {
            motor1.setPower(0);
            motor2.setPower(0);
        }
        robot.telemetry.addData("Intake motor powers: ", "%.5f %.5f",
                motor1.getPower(), motor2.getPower());
    }
}
