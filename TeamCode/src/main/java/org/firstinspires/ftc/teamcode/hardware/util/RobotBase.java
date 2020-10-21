package org.firstinspires.ftc.teamcode.hardware.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotBase {
    private OpMode opMode;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public SubsystemManager subsystemManager = new SubsystemManager();

    public void init(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }
    public abstract void teleopUpdate(); // Only for teleop
    public abstract void update();
    public abstract void end();
}
