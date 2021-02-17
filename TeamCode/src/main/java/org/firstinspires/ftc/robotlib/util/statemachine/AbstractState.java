package org.firstinspires.ftc.robotlib.util.statemachine;

public abstract class AbstractState implements State {

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    /*
     * Should return current state to stay on it
     */
    @Override
    public abstract AbstractState getNextState();

    @Override
    public void end() {

    }
}
