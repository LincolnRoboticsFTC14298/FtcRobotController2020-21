package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.old.Drive;
import org.firstinspires.ftc.teamcode.hardware.util.gamepad.RadicalGamepad;

@TeleOp(name="Drive test", group="Test")
public class DriveTestOpmode extends OpMode {
    Drive drive = new Drive(hardwareMap);
    RadicalGamepad gamepad;


    @Override
    public void init() {
        drive.init();
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        double x = gamepad.left_stick_x, y = gamepad.left_stick_y;
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
