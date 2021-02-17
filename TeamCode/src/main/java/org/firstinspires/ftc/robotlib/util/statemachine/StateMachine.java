package org.firstinspires.ftc.robotlib.util.statemachine;

import android.util.Log;

public class StateMachine {
    private State currentState;

    public StateMachine(State startingState) {
        currentState = startingState;
    }

    public void update() {
        if (currentState == null) {
            Log.e("State machine" , "current state is null.");
            return;
        }

        State nextState = currentState.getNextState();
        if (currentState != nextState) {
            // Different state //

            // End current state
            currentState.end();
            // Assign new state
            currentState = nextState;
            // Start the new state
            currentState.start();
        }
        currentState.update();
    }

    public State getState() {
        return currentState;
    }
}
