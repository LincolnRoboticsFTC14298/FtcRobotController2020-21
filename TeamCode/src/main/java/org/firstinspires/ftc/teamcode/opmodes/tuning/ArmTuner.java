package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import org.firstinspires.ftc.teamcode.robotlib.hardware.gamepad.RadicalGamepad;

@Config
@TeleOp(name="Arm Tuner", group="Tuner")
public class ArmTuner extends OpMode {
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
            robot.arm.openClaw();
        } else if (gamepad.b) {
            robot.arm.closeClaw();
        } else if (gamepad.x) {
            robot.arm.lift();
        } else if (gamepad.y) {
            robot.arm.defaultPos();
        } else if (gamepad.right_bumper) {
            robot.arm.lower();
        }

        robot.update();
    }
}
