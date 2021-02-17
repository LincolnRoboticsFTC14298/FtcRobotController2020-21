package org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine;

import org.firstinspires.ftc.robotlib.util.statemachine.AbstractState;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.util.Field;

public class Collecting extends AbstractNavigationState {

    public Collecting(MainTeleOp teleOp) {
        super(teleOp);
    }

    @Override
    public void update() {
        Field.ringProvider.update(getRobot().localizer.getPoseEstimate()); // may have to be in main update loop

        switch (getTeleOp().getControlMode()) {
            case MANUAL_COLLECTING:
                if (getRobot().drive.isBusy()) {
                    getRobot().drive.cancelFollowing();
                }
                break;
            case FULLY_AUTOMATIC:
                if (!getRobot().drive.isBusy() && Field.ringProvider.getRings().size() > 0) {
                    getRobot().drive.goToRingAsync();
                } else if (!getRobot().drive.isBusy()) {
                    // TODO: rotate to find rings
                }
                break;
        }
    }

    @Override
    public AbstractState getNextState() {
        if (getTeleOp().getControlMode() == MainTeleOp.ControlMode.FULLY_MANUAL) {
            return new Manual(getTeleOp());
        } else if (getTeleOp().getRuntime() > 90 && Field.wobbleGoalProvider.amount() > 0) {
            // Last 30 seconds of match
            return new TravelingToWobbleGoal(getTeleOp());
        } else if (getRobot().ringCounter.getNumberOfRings() == 3) {
            return new TravelingToShoot(getTeleOp());
        }
        return this;
    }

    @Override
    public void end() {
        getRobot().drive.cancelFollowing();
    }
}
