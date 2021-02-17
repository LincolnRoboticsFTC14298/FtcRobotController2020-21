package org.firstinspires.ftc.teamcode.opmodes.testing.pixel2xl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;

@TeleOp(name="Pixel 2xl Vision", group="Tuner")
public class VisionTest extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private Vision vision;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);
    }

    @Override
    public void start() {
        vision.start();
        //vision.setWatershed(true);
    }

    // TODO: Make tuning automatic w/ gradient decent
    @Override
    public void loop() {
        vision.update();
        vision.updateTelemetry();
        TelemetryPacket packet = new TelemetryPacket();
        packet.putAll(vision.getTelemetryData());
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Viewport: ", vision.getViewport());
        telemetry.update();
    }
}

