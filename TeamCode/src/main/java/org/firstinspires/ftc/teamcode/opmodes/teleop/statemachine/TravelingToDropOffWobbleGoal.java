package org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine;

import org.firstinspires.ftc.robotlib.util.statemachine.AbstractState;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.util.Field;

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
            if (Field.wobbleGoalProvider.amount() > 0) {
                return new TravelingToWobbleGoal(getTeleOp());
            } else {
                getRobot().arm.closeClaw();
                return new Collecting(getTeleOp());
            }
        }
        return this;
    }

    @Override
    public void end() {
        getRobot().arm.openClaw();
        // May need to sleep or some IS_DROPPING state
        getRobot().arm.lower();
    }
}
