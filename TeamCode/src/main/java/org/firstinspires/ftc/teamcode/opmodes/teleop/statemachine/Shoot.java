package org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine;

import org.firstinspires.ftc.robotlib.util.statemachine.AbstractState;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;

public class Shoot extends AbstractNavigationState {

    public Shoot(MainTeleOp teleOp) {
        super(teleOp);
    }

    @Override
    public void start() {
        if (getTeleOp().getRuntime() > 90) {
            getRobot().powerShot();
        } else {
            getRobot().shootAsync(3);
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
