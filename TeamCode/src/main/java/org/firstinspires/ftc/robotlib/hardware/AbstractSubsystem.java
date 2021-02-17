package org.firstinspires.ftc.robotlib.hardware;

import org.firstinspires.ftc.robotlib.util.TelemetryData;

import java.util.Map;


public abstract class AbstractSubsystem implements Subsystem {
    private String name;
    protected TelemetryData telemetry;

    public AbstractSubsystem(String name) {
        this.name = name;
        telemetry = new TelemetryData(name);
    }

    public void init() {

    }

    public void initUpdate() {

    }

    public void start() {

    }

    public abstract void update();

    public abstract void stop();

    public void updateMotorAndServoValues() {

    }

    public void updateSensorValues() {

    }

    public void updateTelemetry() {

    }

    public Map<String, Object> getTelemetryData() {
        return telemetry.getData();
    }

    public void updateLogging() {

    }
}
