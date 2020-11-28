package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.RadicalOpMode;

@TeleOp(name="Main TeleOp", group="TeleOp")
public class MainTeleOp extends RadicalOpMode {
    Robot robot = new Robot(this);
    OperatorInterface operatorInterface = new OperatorInterface(robot, gamepad1, gamepad2);

    @Override
    public void init() {
        robot.setAlliance(getAlliance());
        robot.drive.setPoseEstimate(getLastPose());
        robot.init();
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
