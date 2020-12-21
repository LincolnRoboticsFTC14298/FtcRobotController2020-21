package robotlib.hardware;

import java.util.ArrayList;

public class SubsystemManager {
    private ArrayList<Subsystem> subsystems = new ArrayList<>();

    public void add(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void init() {
        for (Subsystem subsystem : subsystems) {
            subsystem.getSensorValues();
            subsystem.init();
            subsystem.updateMotorAndServoValues();
        }
    }

    public void initUpdate() {
        for (Subsystem subsystem : subsystems) {
            subsystem.getSensorValues();
            subsystem.initUpdate();
            subsystem.updateMotorAndServoValues();
        }
    }

    public void start() {
        for (Subsystem subsystem : subsystems) {
            subsystem.getSensorValues();
            subsystem.start();
            subsystem.updateMotorAndServoValues();
        }
    }

    public void update() {
        for (Subsystem subsystem : subsystems) {
            subsystem.getSensorValues();
            subsystem.update();
            subsystem.updateMotorAndServoValues();
        }
    }

    public void stop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.getSensorValues();
            subsystem.stop();
            subsystem.updateMotorAndServoValues();
        }
    }
}
