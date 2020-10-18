package org.firstinspires.ftc.teamcode.compenents;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.Component;
import org.firstinspires.ftc.teamcode.util.RobotMap;

public class Intake implements Component {
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
    public void teleopPeriodic() {
        if (robot.oi.intakeOn()) {
            turnOn();
        }
        if (robot.oi.intakeOff()) {
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
