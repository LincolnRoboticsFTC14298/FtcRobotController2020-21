package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

@TeleOp(name="Arm PID Tuner 2", group="Tuner")
@Config
public class ArmPIDTuner2 extends OpMode {
    private SubsystemManager subsystemManager = new SubsystemManager();
    private Arm arm;
    private RadicalGamepad gamepad;

    private Telemetry telemetry1;

    private double maxVelocity = 0, currentVelocity;
    private double p, i, d, f;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
        subsystemManager.add(arm);
        gamepad = new RadicalGamepad(gamepad1);

        //arm.init();
        arm.resetDefaultAngle();

        telemetry1 = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        gamepad.update();

        if (gamepad.a) {
            arm.openClaw();
        } else if (gamepad.b) {
            arm.closeClaw();
        }

        currentVelocity = arm.getMotorVelocity();

        if (currentVelocity > maxVelocity) {
            maxVelocity = currentVelocity;
        }

        f = 32767.0 / maxVelocity;
        p = .1 * f;
        i = .1 * p;
        d = 0;

        telemetry1.addData("current velocity", currentVelocity);
        telemetry1.addData("maximum velocity", maxVelocity);
        telemetry1.addData("p", p);
        telemetry1.addData("i", i);
        telemetry1.addData("d", d);
        telemetry1.addData("f", f);
        telemetry1.update();
    }
}
