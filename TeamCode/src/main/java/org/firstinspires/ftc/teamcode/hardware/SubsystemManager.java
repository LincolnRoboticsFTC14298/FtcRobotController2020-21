package org.firstinspires.ftc.teamcode.hardware;

import java.util.ArrayList;

public class SubsystemManager {
    private ArrayList<Subsystem> components = new ArrayList<>();

    public void add(Subsystem component) {
        components.add(component);
    }

    public void init() {
        for (Subsystem component : components) {
            component.init();
        }
    }

    public void update() {
        for (Subsystem component : components) {
            component.update();
        }
    }

    public void end() {
        for (Subsystem component : components) {
            component.end();
        }
    }
}
