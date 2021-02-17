package org.firstinspires.ftc.robotlib.vision;

import org.firstinspires.ftc.robotlib.util.TelemetryData;

import java.util.Map;

public abstract class AbstractVisionScorer implements VisionScorer {
    private String name;
    protected TelemetryData telemetry;

    public AbstractVisionScorer(String name) {
        this.name = name;
        telemetry = new TelemetryData(name);
    }

    public void updateTelemetry() {

    }

    public Map<String, Object> getTelemetryData() {
        return telemetry.getData();
    }

    public void updateLogging() {

    }
}
