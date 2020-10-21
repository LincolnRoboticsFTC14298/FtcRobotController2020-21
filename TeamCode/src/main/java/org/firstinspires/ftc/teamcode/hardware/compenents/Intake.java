package org.firstinspires.ftc.teamcode.hardware.compenents;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.RobotMap;

public class Intake implements Subsystem {
    Robot robot;

    DcMotor motor;
    boolean on = false;

    public Intake(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        motor = robot.hardwareMap.get(DcMotor.class, RobotMap.INTAKE_MOTOR_NAME);
        turnOff();
    }

    @Override
    public void update() {
        if (robot.OperatorInterface.isIntakeOn()) {
            turnOn();
        }
        else {
            turnOff();
        }
        updateIntake();
    }

    @Override
    public void end() {
        turnOff();
        updateIntake();
    }

    public void turnOn() {
        on = true;
    }

    public void turnOff() {
        on = false;
    }

    public void updateIntake() {
        if (on) {
            motor.setPower(RobotMap.INTAKE_POWER_ON);
        } else {
            motor.setPower(0);
        }
        robot.telemetry.addData("Intake motor power: ", "%.5f", motor.getPower());
    }
}
