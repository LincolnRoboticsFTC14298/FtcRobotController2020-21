package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;

@TeleOp(name="Radical Gamepad test", group="Test")
public class RadicalGamepadTest extends OpMode {
    RadicalGamepad gamepad;
    int at = 0;

    @Override
    public void init() {
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.update();
        if (gamepad.a) at++;
        telemetry.addData("Times a has been pressed: ", at);
        telemetry.addData("lx: ", gamepad.left_stick_y);
        telemetry.addData("ry: ", gamepad.right_stick_y);
        telemetry.update();
    }
}
