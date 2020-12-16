package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Field;

import robotlib.hardware.gamepad.RadicalGamepad;

@Config
@TeleOp(name="Turret Tuner", group="Tuner")
public class TurretTuner extends OpMode {
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
        robot.setPoseEstimate(new Pose2d(0,0,0));
        robot.setTarget(Field.Target.HIGH_GOAL);
    }

    @Override
    public void loop() {
        gamepad.update();

        Pose2d input = new Pose2d(-gamepad.left_stick_y, -gamepad.left_stick_x, gamepad.right_stick_x);
        robot.drive.teleopControl(input, true, true);

        if (gamepad.a) {
            robot.turret.aimAtTargetAsync();
        } else if (gamepad.b) {
            robot.turret.stopAiming();
        }

        robot.update();

        telemetry.addData("Done aiming: ", robot.turret.isAligned());
        telemetry.update();
    }
}
