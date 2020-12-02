package robotlib.hardware;

import java.util.ArrayList;

public class SubsystemManager {
    private ArrayList<Subsystem> subsystems = new ArrayList<>();

    public void add(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void init_loop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.init_loop();
            subsystem.updateMotorsAndServos();
        }
    }

    public void start() {
        for (Subsystem subsystem : subsystems) {
            subsystem.start();
            subsystem.updateMotorsAndServos();
        }
    }

    public void update() {
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
            subsystem.updateMotorsAndServos();
        }
    }

    public void stop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
            subsystem.updateMotorsAndServos();
        }
    }
}
