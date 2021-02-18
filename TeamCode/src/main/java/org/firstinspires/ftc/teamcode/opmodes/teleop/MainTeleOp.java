package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.util.statemachine.State;
import org.firstinspires.ftc.robotlib.util.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.DataWriterUtil;
import org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine.Collecting;

@TeleOp(name="Main TeleOp", group="TeleOp")
@Disabled
public class MainTeleOp extends OpMode {
    private Robot robot;
    private OperatorInterface operatorInterface = new OperatorInterface(this, robot, gamepad1, gamepad2);

    public enum ControlMode {
        FULLY_MANUAL,
        MANUAL_COLLECTING,
        FULLY_AUTOMATIC
    }

    ControlMode controlMode = ControlMode.MANUAL_COLLECTING;

    private StateMachine navigationStateMachine;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);

        robot.setAlliance(DataWriterUtil.getAlliance());
        robot.setPoseEstimate(DataWriterUtil.getLastPose());

        robot.init();

        navigationStateMachine = new StateMachine(new Collecting(this));
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        operatorInterface.update();

        robot.vision.scan();

        updateNavigation();

        robot.update();

        // TODO: Add telemetry
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public void updateNavigation() {
        navigationStateMachine.update();
    }

    public void setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public Robot getRobot() {
        return robot;
    }

    public State getNavigationState() {
        return navigationStateMachine.getState();
    }
}
