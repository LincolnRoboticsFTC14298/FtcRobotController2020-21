package org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine;

import org.firstinspires.ftc.robotlib.util.statemachine.AbstractState;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;

public class Shooting extends AbstractNavigationState {

    public Shooting(MainTeleOp teleOp) {
        super(teleOp);
    }

    @Override
    public void start() {
        if (getTeleOp().getRuntime() > 90) {
            getRobot().shootAsync(3);
        } else {
            getRobot().powerShot();
        }
    }

    @Override
    public AbstractState getNextState() {
        if (getTeleOp().getControlMode() == MainTeleOp.ControlMode.FULLY_MANUAL) {
            return new Manual(getTeleOp());
        } else if (getRobot().doneShooting()) {
            return new Collecting(getTeleOp());
        }
        return this;
    }
}
