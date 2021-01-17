package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import org.firstinspires.ftc.teamcode.robotlib.hardware.gamepad.RadicalGamepad;

@Config
@TeleOp(name="Intake Tuner", group="Tuner")
public class IntakeTuner extends OpMode {
    private Robot robot;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        robot = new Robot(this);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        gamepad.update();

        if (gamepad.a) {
            robot.intake.turnOn();
        } else if (gamepad.b) {
            robot.intake.turnOff();
        }

        robot.update();
    }
}
