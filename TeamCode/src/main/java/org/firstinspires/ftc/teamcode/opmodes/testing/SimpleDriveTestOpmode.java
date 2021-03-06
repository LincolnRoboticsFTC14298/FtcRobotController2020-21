package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.SimpleDrive;

@TeleOp(name="Simple Drive test", group="Test")
@Disabled
public class SimpleDriveTestOpmode extends OpMode {
    private SimpleDrive drive;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        drive = new SimpleDrive(hardwareMap);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void start() {
        drive.start();
    }

    @Override
    public void loop() {
        gamepad.update();
        double x = gamepad.left_stick_x, y = -gamepad.left_stick_y;
        double radius = Math.hypot(x, y);
        double angle = Math.atan2(y,x);
        drive.teleopControl(radius, angle, gamepad.right_stick_x, 0, false);
        drive.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
