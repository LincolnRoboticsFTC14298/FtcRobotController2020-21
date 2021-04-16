package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

@TeleOp(name="Arm Test", group="Tuner")
public class ArmTest extends OpMode {
    private SubsystemManager subsystemManager = new SubsystemManager();
    private Arm arm;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
        subsystemManager.add(arm);
        gamepad = new RadicalGamepad(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        arm.defaultPos();
    }

    @Override
    public void loop() {
        gamepad.update();

        if (gamepad.a) {
            arm.closeClaw();
        } else if (gamepad.b) {
            arm.openClaw();
        } else if (gamepad.x) {
            arm.lift();
        } else if (gamepad.y) {
            arm.lower();
        } else if (gamepad.right_bumper) {
            arm.middle();
        }

        subsystemManager.update();
    }
}
