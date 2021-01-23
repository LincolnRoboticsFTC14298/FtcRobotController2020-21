package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.Drive;

@TeleOp(name="Drive test", group="Test")
@Disabled
public class DriveTestOpmode extends OpMode {
    Drive drive;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
    }

    @Override
    public void start() {
        drive.start();
    }

    @Override
    public void loop() {
        Pose2d input = new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad2.right_stick_x);
        drive.teleopControl(input, false, false);
        drive.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
