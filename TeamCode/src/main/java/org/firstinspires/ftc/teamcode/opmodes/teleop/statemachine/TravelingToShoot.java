package org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine;

import org.firstinspires.ftc.robotlib.util.statemachine.AbstractState;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;

public class TravelingToShoot extends AbstractNavigationState {

    public TravelingToShoot(MainTeleOp teleOp) {
        super(teleOp);
    }

    @Override
    public void start() {
        getRobot().drive.goBehindLineAsync();
    }

    @Override
    public AbstractState getNextState() {
        if (getTeleOp().getControlMode() == MainTeleOp.ControlMode.FULLY_MANUAL) {
            return new Manual(getTeleOp());
        } else if (!getRobot().drive.isBusy() && getRobot().drive.isBehindLine()) {
            return new Shooting(getTeleOp());
        }
        return this;
    }
}
