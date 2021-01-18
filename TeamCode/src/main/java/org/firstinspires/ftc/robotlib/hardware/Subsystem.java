package org.firstinspires.ftc.robotlib.hardware;

import org.firstinspires.ftc.robotlib.util.TelemetryData;

import java.util.Map;


public abstract class Subsystem {
    private String name;
    protected TelemetryData telemetry;

    public Subsystem(String name) {
        this.name = name;
        telemetry = new TelemetryData(name);
    }

    public abstract void update();
    public abstract void stop();

    public void updateMotorAndServoValues() {

    }

    public void getSensorValues() {

    }

    public void init() {

    }

    public void initUpdate() {

    }

    public void start() {

    }

    public void updateTelemetry() {

    }
    public Map<String, Object> getTelemetryData() {
        return telemetry.getData();
    }

    public void updateLogging() {

    }
}
