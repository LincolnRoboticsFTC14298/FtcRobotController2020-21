package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

@TeleOp(name="Arm PID Tuner", group="Tuner")
public class ArmPIDTuner extends OpMode {
    private SubsystemManager subsystemManager = new SubsystemManager();
    private Arm arm;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
        subsystemManager.add(arm);
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
            arm.openClaw();
        } else if (gamepad.b) {
            arm.closeClaw();
        } else if (gamepad.x) {
            arm.lift();
        } else if (gamepad.y) {
            arm.defaultPos();
        } else if (gamepad.right_bumper) {
            arm.lower();
        }

        subsystemManager.update();
    }
}
