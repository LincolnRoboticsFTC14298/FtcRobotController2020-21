package org.firstinspires.ftc.robotlib.util.statemachine;

public abstract class State {
    public void start() {

    }

    public void update() {

    }

    /*
     * Should return current state to stay on it
     */
    public abstract State getState();

    public void end() {

    }
}
