package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

@TeleOp(name="Arm Tuner", group="Tuner")
public class ArmTuner extends OpMode {
    private SubsystemManager subsystemManager = new SubsystemManager();
    private Arm arm;
    private RadicalGamepad gamepad;

    public static double incrClaw = 0.005;
    public static double incrArm = 0.005;

    private double clawPos = .5;
    private double armAngle = 1;

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
            clawPos += incrClaw;
        } else if (gamepad.b) {
            clawPos -= incrClaw;
        } else if (gamepad.x) {
            armAngle += incrArm;
        } else if (gamepad.y) {
            armAngle -= incrArm;
        } else if (gamepad.left_bumper) {
            arm.resetDefaultAngle(armAngle);
        } else if (gamepad.right_bumper) {
            arm.defaultPos();
        }

        clawPos = Range.clip(clawPos, 0, 1);
        armAngle = Range.clip(armAngle, 0, Math.PI * 2 / 3.0);

        arm.setClawPosition(clawPos);
        arm.setArmAngle(armAngle);

        telemetry.addData("Claw Pos", clawPos);
        telemetry.addData("Arm Angle", armAngle);

        subsystemManager.update();
    }
}
