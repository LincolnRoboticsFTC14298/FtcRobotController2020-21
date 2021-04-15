package org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine;

import org.firstinspires.ftc.robotlib.util.statemachine.AbstractState;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;

public class TravelingToDropOffWobbleGoal extends AbstractNavigationState {

    public TravelingToDropOffWobbleGoal(MainTeleOp teleOp) {
        super(teleOp);
    }

    @Override
    public void start() {
        getRobot().drive.goToWallAsync();
    }

    @Override
    public AbstractState getNextState() {
        if (getTeleOp().getControlMode() == MainTeleOp.ControlMode.FULLY_MANUAL) {
            return new Manual(getTeleOp());
        } else if (!getRobot().drive.isBusy() && getRobot().drive.isAtWall()) {
            return new Collecting(getTeleOp());
        }
        return this;
    }

    @Override
    public void end() {
        // Drop wobble goal
        getRobot().arm.middle();
        // May need to sleep or some IS_DROPPING state
        getRobot().arm.openClaw();
    }
}
