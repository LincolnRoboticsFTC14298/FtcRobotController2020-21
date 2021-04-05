package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.Drive;

@TeleOp(name="Drive test", group="Test")
@Disabled
public class DriveTest extends OpMode {
    private SubsystemManager subsystemManager = new SubsystemManager();
    private Drive drive;
    private RadicalGamepad gamepad;

    private boolean fieldCentric = false;
    private boolean autoAim = false;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        subsystemManager.add(drive);
        gamepad = new RadicalGamepad(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        drive.start();
    }

    @Override
    public void loop() {
        gamepad.update();
        if (gamepad.a) autoAim = !autoAim;
        if (gamepad.b) fieldCentric = !fieldCentric;
        drive.teleopControl(getInput(), fieldCentric, autoAim);
        drive.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }

    public Pose2d getInput() {
        return new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad2.right_stick_x);
    }
}
