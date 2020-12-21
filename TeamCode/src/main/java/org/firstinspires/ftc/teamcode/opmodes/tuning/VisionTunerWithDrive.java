package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline.Viewport;

import robotlib.hardware.gamepad.RadicalGamepad;

@TeleOp(name="Vision With Drive", group="Tuner")
public class VisionTunerWithDrive extends OpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Drive drive;
    private Vision vision;
    private RadicalGamepad gamepad;
    
    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        vision = new Vision(hardwareMap);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void start() {
        drive.start();
        vision.start();
        vision.startStreaming();
    }

    @Override
    public void loop() {
        gamepad.update();

        Pose2d input = new Pose2d(-gamepad.left_stick_y, -gamepad.left_stick_x, gamepad.right_stick_x);
        drive.teleopControl(input, false, false);

        if (gamepad.dpad_down) {
            vision.setViewport(Viewport.RAW_MASK);
        } else if (gamepad.dpad_left) {
            vision.setViewport(Viewport.MASKED);
        } else if (gamepad.dpad_up) {
            vision.setViewport(Viewport.DIST1);
        } else if (gamepad.dpad_right) {
            vision.setViewport(Viewport.DIST2);
        } else if (gamepad.a) {
            vision.setViewport(Viewport.MARKERS);
        } else if (gamepad.b) {
            vision.setViewport(Viewport.RAW_IMAGE);
        }

        if (gamepad.x) {
            vision.saveOutput();
        }

        drive.update();
        drive.updateMotorAndServoValues();
        vision.update();

        telemetry.addData("Viewport: ", vision.getViewport());
        telemetry.update();
        dashboard.sendImage(vision.getOutput());
    }
}
