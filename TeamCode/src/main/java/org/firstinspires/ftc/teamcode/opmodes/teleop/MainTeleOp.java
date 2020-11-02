package org.firstinspires.ftc.teamcode.opmodes.teleop;

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
        operatorInterface.init(gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        operatorInterface.update();
        robot.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
