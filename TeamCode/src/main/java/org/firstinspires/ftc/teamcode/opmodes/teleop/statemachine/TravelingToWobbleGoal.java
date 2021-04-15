package org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine;

import org.firstinspires.ftc.robotlib.util.statemachine.AbstractState;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.util.Field;

public class TravelingToWobbleGoal extends AbstractNavigationState {

    public TravelingToWobbleGoal(MainTeleOp teleOp) {
        super(teleOp);
    }

    @Override
    public void start() {
        getRobot().intake.turnOff();

        getRobot().arm.lower();
        getRobot().arm.openClaw();
        getRobot().drive.goToWobbleGoalAsync();
    }

    @Override
    public AbstractState getNextState() {
        if (getTeleOp().getControlMode() == MainTeleOp.ControlMode.FULLY_MANUAL) {
            return new Manual(getTeleOp());
        } else if (!getRobot().drive.isBusy() && getRobot().drive.isAtWobbleGoal()) {
            Field.wobbleGoalProvider.update(getRobot().localizer.getPoseEstimate()); // Remove closest wobble goal
            return new TravelingToDropOffWobbleGoal(getTeleOp());
        }
        return this;
    }

    @Override
    public void end() {
        getRobot().arm.closeClaw();
        // May need to sleep or some PICKING_UP state
        getRobot().arm.lift();
    }
}
