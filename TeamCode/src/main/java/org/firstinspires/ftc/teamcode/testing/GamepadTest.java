package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.util.gamepad.RadicalGamepad;

@TeleOp(name="Gamepad test", group="Test")
public class GamepadTest extends OpMode {
    RadicalGamepad gamepad;


    @Override
    public void init() {
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.update();
        telemetry.addData("a button = ", gamepad.a);
    }

    @Override
    public void stop() {

    }
}
