package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.OperatorInterface;

@TeleOp(name="Main TeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {
    Robot robot = new Robot();
    OperatorInterface operatorInterface = new OperatorInterface(robot);

    @Override
    public void init() {
        robot.init(this);
        operatorInterface.init(this);
    }

    @Override
    public void loop() {
        operatorInterface.update();
        robot.teleopUpdate();
        robot.update();
    }

    @Override
    public void stop() {
        robot.end();
    }
}
