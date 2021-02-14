package org.firstinspires.ftc.robotlib.util.statemachine;

import android.util.Log;

public class StateMachine {
    private State currentState;

    public StateMachine() {

    }
    public StateMachine(State startingState) {
        currentState = startingState;
    }

    public void update() {
        if (currentState == null) {
            Log.e("State machine" , "current state is null.");
            return;
        }

        State newState = currentState.getState();
        if (currentState != newState) {
            // Different state //

            // End current state
            currentState.end();
            // Assign new state
            currentState = newState;
            // Start the new state
            currentState.start();
        }
        currentState.update();
    }

    public State getState() {
        return currentState;
    }
    public void setState(State state) {
        currentState = state;
    }
}
