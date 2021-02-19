package org.firstinspires.ftc.robotlib.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;

public class SubsystemManager {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private ArrayList<Subsystem> subsystems = new ArrayList<>();

    public void add(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void init() {
        dashboard.setTelemetryTransmissionInterval(25);

        for (Subsystem subsystem : subsystems) {
            subsystem.readSensorValues();
            subsystem.init();
            subsystem.updateMotorAndServoValues();
            subsystem.updateTelemetry();
            subsystem.updateLogging();
        }
        pushTelemetry();
    }

    public void initUpdate() {
        for (Subsystem subsystem : subsystems) {
            subsystem.readSensorValues();
            subsystem.initUpdate();
            subsystem.updateMotorAndServoValues();
            subsystem.updateTelemetry();
            subsystem.updateLogging();
        }
        pushTelemetry();
    }

    public void start() {
        for (Subsystem subsystem : subsystems) {
            subsystem.readSensorValues();
            subsystem.start();
            subsystem.updateMotorAndServoValues();
            subsystem.updateTelemetry();
            subsystem.updateLogging();
        }
        pushTelemetry();
    }

    public void update() {
        for (Subsystem subsystem : subsystems) {
            subsystem.readSensorValues();
            subsystem.update();
            subsystem.updateMotorAndServoValues();
            subsystem.updateTelemetry();
            subsystem.updateLogging();
        }
        pushTelemetry();
    }

    public void stop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.readSensorValues();
            subsystem.stop();
            subsystem.updateMotorAndServoValues();
            subsystem.updateTelemetry();
            subsystem.updateLogging();
        }
        pushTelemetry();
    }

    private void pushTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        for (Subsystem subsystem : subsystems) {
            packet.putAll(subsystem.getTelemetryData());
        }
        dashboard.sendTelemetryPacket(packet);
    }
}
