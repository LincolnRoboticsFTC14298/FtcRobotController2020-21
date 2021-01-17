package org.firstinspires.ftc.teamcode.robotlib.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotBase {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public SubsystemManager subsystemManager = new SubsystemManager();

    public RobotBase(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public abstract void init();
    public abstract void initUpdate();
    public abstract void start();
    public abstract void update();
    public abstract void stop();
}
