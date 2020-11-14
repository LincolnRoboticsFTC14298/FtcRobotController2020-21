package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.hardware.util.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline.Viewport;

@TeleOp(name="Camera With Drive", group="Tuner")
public class CameraTunerWithoutDrive extends OpMode {
    private FtcDashboard dashboard;
    private Vision vision = new Vision();
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        vision.init(hardwareMap);
        vision.startStreaming();
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.update();

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

        telemetry.addData("Viewport: ", vision.getViewport());
        dashboard.sendImage(vision.getOutput());
    }
}
