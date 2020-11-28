package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import robotlib.hardware.Subsystem;

@Config
public class Intake implements Subsystem {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    //private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String FRONT_NAME = "intakeFront";
    private static final String REAR_NAME = "intakeRear";

    public static double FRONT_POWER_ON = 1;
    public static double REAR_POWER_ON = 1;

    private DcMotorEx front, rear;
    private double frontPower = 0, rearPower = 0;

    public Intake(HardwareMap hardwareMap) {
        front = hardwareMap.get(DcMotorEx.class, FRONT_NAME);
        rear = hardwareMap.get(DcMotorEx.class, REAR_NAME);
        rear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init() {
        turnOff();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        turnOff();
    }

    public void turnOn() {
        frontPower = FRONT_POWER_ON;
        rearPower = REAR_POWER_ON;
    }
    public void turnOff() {
        frontPower = 0;
        rearPower = 0;
    }

    @Override
    public void updateMotorsAndServos() {
        front.setPower(frontPower);
        rear.setPower(rearPower);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Front intake motor power: ", front.getPower());
        packet.put("Back intake motor power: ", rear.getPower());
        dashboard.sendTelemetryPacket(packet);
    }
}
