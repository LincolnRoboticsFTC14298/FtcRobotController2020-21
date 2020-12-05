package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import robotlib.hardware.gamepad.Button;
import robotlib.hardware.gamepad.ButtonType;
import robotlib.hardware.gamepad.ToggleButton;

@TeleOp(name="Button test", group="Test")
public class ButtonTest extends OpMode {
    Button b;
    ToggleButton x, y;
    int bt = 0;


    @Override
    public void init() {
        b = new Button(gamepad1, ButtonType.b);
        x = new ToggleButton(gamepad1, ButtonType.x);
        y = new ToggleButton(gamepad1, ButtonType.y, 5);
    }

    @Override
    public void loop() {
        if (b.isPressed()) bt++;
        telemetry.addData("Times b has been pressed: ", bt);
        telemetry.addData("Toggle x = ", x.on());
        telemetry.addData("Debounce toggle y w/ 5 sec debounce = ", y.on());
        telemetry.update();
    }
}
