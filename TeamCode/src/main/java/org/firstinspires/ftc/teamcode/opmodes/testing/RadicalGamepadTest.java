package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;

@TeleOp(name="Radical Gamepad test", group="Test")
@Disabled
public class RadicalGamepadTest extends OpMode {
    private RadicalGamepad gamepad;
    private int at = 0, bt = 0, xt = 0, yt = 0;
    private int dpad_upt = 0, dpad_downt = 0, dpad_leftt = 0, dpad_rightt = 0;
    private int left_bumpert = 0, right_bumpert = 0;
    private int left_stick_buttont = 0, right_stick_buttont = 0;
    private int backt = 0, guidet = 0, startt = 0;

    @Override
    public void init() {
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.update();
        if (gamepad.a) at++;
        if (gamepad.b) bt++;
        if (gamepad.x) xt++;
        if (gamepad.y) yt++;
        if (gamepad.dpad_up) dpad_upt++;
        if (gamepad.dpad_down) dpad_downt++;
        if (gamepad.dpad_left) dpad_leftt++;
        if (gamepad.dpad_right) dpad_rightt++;
        if (gamepad.left_bumper) left_bumpert++;
        if (gamepad.right_bumper) right_bumpert++;
        if (gamepad.left_stick_button) left_stick_buttont++;
        if (gamepad.right_stick_button) right_stick_buttont++;
        if (gamepad.back) backt++;
        if (gamepad.guide) guidet++;
        if (gamepad.start) startt++;

        telemetry.addData("Times a has been pressed", at);
        telemetry.addData("Times b has been pressed", bt);
        telemetry.addData("Times x has been pressed", xt);
        telemetry.addData("Times y has been pressed", yt);
        telemetry.addLine();
        telemetry.addData("Times dpad_up has been pressed", dpad_upt);
        telemetry.addData("Times dpad_down has been pressed", dpad_downt);
        telemetry.addData("Times dpad_left has been pressed", dpad_leftt);
        telemetry.addData("Times dpad_right has been pressed", dpad_rightt);
        telemetry.addLine();
        telemetry.addData("Times left_bumper has been pressed", left_bumpert);
        telemetry.addData("Times right_bumper has been pressed", right_bumpert);
        telemetry.addLine();
        telemetry.addData("Times left_stick_button has been pressed", left_stick_buttont);
        telemetry.addData("Times right_stick_button has been pressed", right_stick_buttont);
        telemetry.addLine();
        telemetry.addData("Times back has been pressed", backt);
        telemetry.addData("Times guide has been pressed", guidet);
        telemetry.addData("Times start has been pressed", startt);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("left stick x", gamepad.left_stick_x);
        telemetry.addData("left stick y", gamepad.left_stick_y);
        telemetry.addData("right stick x", gamepad.right_stick_x);
        telemetry.addData("right stick y", gamepad.right_stick_y);
        telemetry.addLine();
        telemetry.addData("left bumper", gamepad.left_trigger);
        telemetry.addData("right bumper", gamepad.right_trigger);
        telemetry.update();
    }
}
