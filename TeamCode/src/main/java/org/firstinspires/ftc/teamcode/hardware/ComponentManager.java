package org.firstinspires.ftc.teamcode.hardware;

import java.util.ArrayList;

public class ComponentManager {
    private ArrayList<Component> components = new ArrayList<>();

    public void add(Component component) {
        components.add(component);
    }

    public void init() {
        for (Component component : components) {
            component.init();
        }
    }

    public void periodic() {
        for (Component component : components) {
            component.teleopPeriodic();
        }
    }

    public void end() {
        for (Component component : components) {
            component.end();
        }
    }
}
