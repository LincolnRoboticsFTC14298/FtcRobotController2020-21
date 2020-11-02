package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;

@Config
public class Intake implements Subsystem {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet;

    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String FRONT_NAME = "intakeFront";
    private static final String REAR_NAME = "intakeRear";

    public static double FRONT_POWER_ON = 1;
    public static double REAR_POWER_ON = 1;

    private DcMotorEx front, rear;
    private double frontPower = 0, rearPower = 0;
    private boolean on = false;

    public Intake() {

    }

    @Override
    public void init(HardwareMap hardwareMap) {
        packet = new TelemetryPacket();

        front = hardwareMap.get(DcMotorEx.class, FRONT_NAME);
        rear = hardwareMap.get(DcMotorEx.class, REAR_NAME);

        rear.setDirection(DcMotorSimple.Direction.REVERSE);

        turnOff();
        updateIntakeMotors();

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void update() {
        packet = new TelemetryPacket();

        updateIntakeMotors();

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        packet = new TelemetryPacket();

        turnOff();
        updateIntakeMotors();

        dashboard.sendTelemetryPacket(packet);
    }

    public void turnOn() {
        frontPower = FRONT_POWER_ON;
        rearPower = REAR_POWER_ON;
    }
    public void turnOff() {
        frontPower = 0;
        rearPower = 0;
    }

    public void updateIntakeMotors() {
        front.setPower(frontPower);
        rear.setPower(rearPower);
        packet.put("Front intake motor power: ", front.getPower());
        packet.put("Back intake motor power: ", rear.getPower());
    }
}
