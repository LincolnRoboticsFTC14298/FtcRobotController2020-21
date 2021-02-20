package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.vision.RingPipeline.Viewport;

@TeleOp(name="Vision With Drive", group="Tuner")
public class VisionTunerWithDrive extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
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
            vision.setViewport(Viewport.RAW_IMAGE);
        } else if (gamepad.dpad_right) {
            vision.setViewport(Viewport.ANNOTATED);
        }

        if (gamepad.x) {
            vision.saveOutput();
        }

        vision.update();
        vision.updateTelemetry();
        drive.update();
        drive.updateMotorAndServoValues();
        drive.updateTelemetry();

        TelemetryPacket packet = new TelemetryPacket();
        packet.putAll(vision.getTelemetryData());
        packet.putAll(drive.getTelemetryData());
        dashboard.sendTelemetryPacket(packet);
    }
}
