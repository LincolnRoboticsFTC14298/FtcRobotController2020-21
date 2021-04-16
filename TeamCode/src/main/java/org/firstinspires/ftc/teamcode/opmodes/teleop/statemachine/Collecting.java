package org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine;

import org.firstinspires.ftc.robotlib.util.statemachine.AbstractState;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.util.Field;

public class Collecting extends AbstractNavigationState {

    public Collecting(MainTeleOp teleOp) {
        super(teleOp);
    }

    @Override
    public void start() {
        getRobot().intake.turnOn();
        getRobot().arm.defaultPos();
    }

    @Override
    public void update() {
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
                    // rotate to find rings
                }
                break;
        }
    }

    @Override
    public AbstractState getNextState() {
        if (getTeleOp().getControlMode() == MainTeleOp.ControlMode.FULLY_MANUAL) {
            return new Manual(getTeleOp());
        } else if (getTeleOp().getRuntime() > 90 &&
                getRobot().ringCounter.getTotalRings() == 0 &&
                Field.wobbleGoalProvider.amount() > 0) {
            // Last 30 seconds of match and no rings in intake
            return new TravelingToWobbleGoal(getTeleOp());
        } else if (getRobot().ringCounter.getTotalRings() == 3) {
            return new Shoot(getTeleOp());
        }
        return this;
    }

    @Override
    public void end() {
        getRobot().drive.cancelFollowing();
    }
}
