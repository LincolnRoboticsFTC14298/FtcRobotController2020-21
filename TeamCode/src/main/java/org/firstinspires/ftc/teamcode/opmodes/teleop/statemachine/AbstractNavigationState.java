package org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine;

import org.firstinspires.ftc.robotlib.util.statemachine.AbstractState;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;

public abstract class AbstractNavigationState extends AbstractState {

    private final MainTeleOp teleOp;
    private final Robot robot;

    public AbstractNavigationState(MainTeleOp teleOp) {
        this.teleOp = teleOp;
        this.robot = teleOp.getRobot();
    }

    public MainTeleOp getTeleOp() {
        return teleOp;
    }

    public Robot getRobot() {
        return robot;
    }
}
