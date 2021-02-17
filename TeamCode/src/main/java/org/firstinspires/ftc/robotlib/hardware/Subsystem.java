package org.firstinspires.ftc.robotlib.hardware;

import java.util.Map;

public interface Subsystem {
    void init();

    void initUpdate();

    void start();

    void update();

    void stop();

    void updateSensorValues();

    void updateMotorAndServoValues();

    void updateTelemetry();

    Map<String, Object> getTelemetryData();

    void updateLogging();
}
