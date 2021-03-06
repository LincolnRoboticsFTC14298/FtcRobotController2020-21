package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.RingPipeline.Viewport;

@TeleOp(name="Vision Without Drive", group="Tuner")
public class VisionTunerWithoutDrive extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private SubsystemManager subsystemManager = new SubsystemManager();
    private Vision vision;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);
        subsystemManager.add(vision);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void start() {
        vision.start();
        //vision.setWatershed(true);
    }

    // TODO: Make tuning automatic w/ gradient decent
    @Override
    public void loop() {
        gamepad.update();
        vision.analyze();

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

        subsystemManager.update();
    }
}
