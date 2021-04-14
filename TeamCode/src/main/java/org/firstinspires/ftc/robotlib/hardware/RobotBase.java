package org.firstinspires.ftc.robotlib.hardware;

import java.util.Map;

public interface RobotBase {
    void init();
    void initUpdate();
    void start();
    void update();
    void stop();

    void updateTelemetry();
    Map<String, Object> getTelemetry();
    void updateLogging();
}
