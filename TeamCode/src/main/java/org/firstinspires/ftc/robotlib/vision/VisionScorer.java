package org.firstinspires.ftc.robotlib.vision;

import org.firstinspires.ftc.teamcode.vision.RingData;

import java.util.Map;

public interface VisionScorer {
    double score(RingData ringData);

    double getWeight();

    void updateTelemetry();

    Map<String, Object> getTelemetryData();

    void updateLogging();
}
