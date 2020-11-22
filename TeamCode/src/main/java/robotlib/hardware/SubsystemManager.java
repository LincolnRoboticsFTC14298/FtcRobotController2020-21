package robotlib.hardware;

import java.util.ArrayList;

public class SubsystemManager {
    private ArrayList<Subsystem> subsystems = new ArrayList<>();

    public void add(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void init() {
        for (Subsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void update() {
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    public void stop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }
}
