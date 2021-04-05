package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.Button;
import org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonTrigger;
import org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType;
import org.firstinspires.ftc.robotlib.hardware.gamepad.ToggleButton;

@TeleOp(name="Button test", group="Test")
public class ButtonTest extends OpMode {
    Button b;
    ToggleButton x, y;
    ButtonTrigger r, l;
    int bt = 0, xt = 0, rt = 0, st = 0;


    @Override
    public void init() {
        b = new Button(gamepad1, ButtonType.b);
        x = new ToggleButton(gamepad1, ButtonType.x, 1);
        r = new ButtonTrigger(gamepad1, ButtonType.right_trigger);
        l = new ButtonTrigger(gamepad1, ButtonType.left_trigger);
    }

    @Override
    public void loop() {
        if (b.isPressed()) bt++;
        if (x.changedToOn()) xt++;
        if (r.isPressed()) rt++;
        if (r.isPressed() && l.isPressed()) st++;
        telemetry.addData("Times b has been pressed: ", bt);
        telemetry.addData("Toggle x = ", x.isOn());
        telemetry.addData("Times x changed from off to on: ", xt);
        telemetry.addData("Times right trigger button pressed: ", rt);
        telemetry.addData("Times right and left trigger buttons pressed together: ", st);
        telemetry.update();
    }
}
