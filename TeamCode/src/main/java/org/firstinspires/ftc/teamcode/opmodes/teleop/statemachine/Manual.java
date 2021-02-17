package org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine;

import org.firstinspires.ftc.robotlib.util.statemachine.AbstractState;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;

// Navigation //
public class Manual extends AbstractNavigationState {

    public Manual(MainTeleOp teleOp) {
        super(teleOp);
    }

    @Override
    public void start() {
        getRobot().drive.cancelFollowing();
    }

    @Override
    public AbstractState getNextState() {
        switch (getTeleOp().getControlMode()) {
            case MANUAL_COLLECTING:
            case FULLY_AUTOMATIC:
                return new Collecting(getTeleOp());
        }
        return this;
    }
}
