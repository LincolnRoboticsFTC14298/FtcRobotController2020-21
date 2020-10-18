package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.ComponentOpMode;

@TeleOp(name="Main TeleOp", group="TeleOp")
public class MainTeleOp extends ComponentOpMode {
    Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap, this);
    }

    @Override
    public void loop() {
        robot.periodic();
    }

    @Override
    public void stop() {
        robot.end();
    }
}
