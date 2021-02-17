package org.firstinspires.ftc.robotlib.util.statemachine;

public interface State {

    void start();

    void update();

    /*
     * Should return current state to stay on it
     */
    State getNextState();

    void end();
}
