package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.old.Drive;
import robotlib.hardware.gamepad.RadicalGamepad;

@TeleOp(name="Drive test", group="Test")
public class DriveTestOpmode extends OpMode {
    Drive drive;
    RadicalGamepad gamepad;


    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        drive.init();
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        //gamepad.update();
        double x = gamepad1.left_stick_x, y = -gamepad1.left_stick_y;
        double radius = Math.hypot(x, y);
        double angle = Math.atan2(y,x);
        drive.teleopControl(radius, angle, gamepad1.right_stick_x, 0, false);
        drive.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
