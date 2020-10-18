package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.compenents.Drive;
import org.firstinspires.ftc.teamcode.hardware.Component;
import org.firstinspires.ftc.teamcode.hardware.ComponentManager;
import org.firstinspires.ftc.teamcode.hardware.ComponentOpMode;

import org.firstinspires.ftc.teamcode.util.OI;
import org.firstinspires.ftc.teamcode.util.RobotMap;


public class Robot {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public OI oi;
    public Drive drive = new Drive(this);;

    public ComponentManager componentManager = new ComponentManager();

    public void init(HardwareMap hardwareMap, ComponentOpMode opMode) {
        this.hardwareMap = hardwareMap;

        telemetry = opMode.telemetry;

        oi = new OI(opMode);

        componentManager.add(drive);

        componentManager.init();

        telemetry.update();
    }

    public void periodic() {
        componentManager.periodic();
        telemetry.update();
    }

    public void end() {
        componentManager.end();
        telemetry.update();
    }
}
