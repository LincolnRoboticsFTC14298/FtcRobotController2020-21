package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.util.gamepad.Button;
import org.firstinspires.ftc.teamcode.hardware.util.gamepad.ButtonType;
import org.firstinspires.ftc.teamcode.hardware.util.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.util.gamepad.ToggleButton;

@TeleOp(name="Gamepad test", group="Test")
public class GamepadTest extends OpMode {
    RadicalGamepad gamepad;
    Button b;
    ToggleButton x, y;


    @Override
    public void init() {
        gamepad = new RadicalGamepad(gamepad1);
        b = new Button(gamepad1, ButtonType.b);
        x = new ToggleButton(gamepad1, ButtonType.x);
        y = new ToggleButton(gamepad1, ButtonType.y, 5);
    }

    @Override
    public void loop() {
        gamepad.update();
        telemetry.addData("Gamepad a button = ", gamepad.a);
        telemetry.addData("lx: ", gamepad.left_stick_y);
        telemetry.addData("ry: ", gamepad.right_stick_y);
        telemetry.addLine();
        telemetry.addData("Button b = ", b.isPressed());
        telemetry.addData("Toggle x = ", x.on());
        telemetry.addData("Debounce toggle y w/ 5 sec debounce = ", y.on());
    }

    @Override
    public void stop() {

    }
}
