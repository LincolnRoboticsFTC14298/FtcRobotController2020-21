package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp
@Disabled
public class RingCounterTuner extends OpMode {
    Robot robot;
    RadicalGamepad gamepad;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();

        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        if (gamepad.a) {
            robot.intake.turnOn();
        } else if (gamepad.b) {
            robot.intake.turnOff();
        } else if (gamepad.x) {
            robot.intake.turnOnReverse();
        }
        robot.update();
    }
}
