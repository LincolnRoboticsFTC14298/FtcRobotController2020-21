package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

@Config
@TeleOp(name="Intake Tuner", group="Tuner")
@Disabled
public class IntakeTuner extends OpMode {
    private SubsystemManager subsystemManager = new SubsystemManager();
    private Intake intake;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        subsystemManager.add(intake);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void start() {
        subsystemManager.start();
    }

    @Override
    public void loop() {
        gamepad.update();

        if (gamepad.a) {
            intake.turnOn();
        } else if (gamepad.b) {
            intake.turnOff();
        }

        subsystemManager.update();
    }
}
