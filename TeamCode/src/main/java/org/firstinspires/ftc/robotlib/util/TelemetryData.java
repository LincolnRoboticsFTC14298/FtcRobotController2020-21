package org.firstinspires.ftc.robotlib.util;

import java.util.HashMap;
import java.util.Map;

public class TelemetryData {
    private String prefix;
    private Map<String, Object> data;

    public TelemetryData(String prefix) {
        this.prefix = prefix;
        this.data = new HashMap<>();
    }

    public void put(String label, Object value) {
        data.put(prefix + ": " + label, value);
    }
    public void putAll(Map<String, Object> data) {
        data.forEach(
                (label, value) -> this.data.put(prefix + ": " + label, value)
                );
    }

    public Map<String, Object> getData() {
        return data;
    }
}
