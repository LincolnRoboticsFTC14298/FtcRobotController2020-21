package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RingCounter;

@TeleOp
public class RingCounterTuner extends OpMode {
    private SubsystemManager subsystemManager = new SubsystemManager();
    private RingCounter ringCounter;
    private Intake intake;
    RadicalGamepad gamepad;

    @Override
    public void init() {
        ringCounter = new RingCounter(hardwareMap);
        intake = new Intake(hardwareMap, ringCounter);
        subsystemManager.add(ringCounter, intake);
        subsystemManager.init();

        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.update();

        if (gamepad.a) {
            intake.turnOn();
        } else if (gamepad.b) {
            intake.turnOff();
        } else if (gamepad.x) {
            intake.turnOnReverse();
        }
        subsystemManager.update();
    }
}
