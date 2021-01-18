package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp(name="Elevator Tuner", group="Tuner")
public class ElevatorTuner extends OpMode {
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
            robot.elevator.raise();
        } else if (gamepad.b) {
            robot.elevator.lower();
        }

        robot.update();

        telemetry.addData("IsUp: ", robot.elevator.isUp());
        telemetry.addData("IsDown: ", robot.elevator.isDown());
        telemetry.update();
    }
}
