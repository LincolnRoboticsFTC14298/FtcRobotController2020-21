package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.RingCountPipeline.Viewport;

@TeleOp(name="Vision Without Drive", group="Tuner")
public class VisionTunerWithoutDrive extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private Vision vision;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void start() {
        vision.start();
        //vision.setWatershed(true);
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
        } else if (gamepad.y) {
            vision.setViewport(Viewport.ANNOTATED);
        }

        if (gamepad.x) {
            vision.saveOutput();
        }

        vision.update();
        vision.updateTelemetry();
        TelemetryPacket packet = new TelemetryPacket();
        packet.putAll(vision.getTelemetryData());
        packet.put("Viewport", vision.getViewport().toString());
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Viewport: ", vision.getViewport());
        telemetry.update();
    }
}
