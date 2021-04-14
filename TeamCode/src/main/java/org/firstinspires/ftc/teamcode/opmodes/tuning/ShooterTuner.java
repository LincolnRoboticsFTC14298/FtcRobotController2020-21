package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Field;

@Config
@TeleOp(name="Shooter Tuner", group="Tuner")
@Disabled
public class ShooterTuner extends OpMode {
    private SubsystemManager subsystemManager = new SubsystemManager();
    private Localizer localizer;
    private Shooter shooter;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        localizer = new Localizer(hardwareMap);
        localizer.setPoseEstimate(new Pose2d(0,0,0));
        localizer.setTarget(Field.Target.HIGH_GOAL);
        shooter = new Shooter(hardwareMap, localizer);
        subsystemManager.add(shooter);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        gamepad.update();

        shooter.aimAsync();

        if (gamepad.a) {
            shooter.launch();
        }

        subsystemManager.update();

        telemetry.addData("Is extended", shooter.isExtended());
        telemetry.addData("Is retracted", shooter.isRetracted());
        telemetry.addData("Launch status", shooter.getLaunchStatus().toString());
        telemetry.addData("Is done aiming", shooter.doneAiming());
        telemetry.addData("Is ready to launch", shooter.readyToLaunch());
        telemetry.update();
    }
}
