package org.firstinspires.ftc.robotlib.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

// Similar to acme robotics StickyGamepad
public class RadicalGamepad {
    private Gamepad gamepad;

    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean a, b, x, y;
    public boolean left_bumper, right_bumper;
    public boolean left_stick_button, right_stick_button;

    private org.firstinspires.ftc.robotlib.hardware.gamepad.Button dpadUp, dpadDown, dpadLeft, dpadRight;
    private org.firstinspires.ftc.robotlib.hardware.gamepad.Button A, B, X, Y;
    private org.firstinspires.ftc.robotlib.hardware.gamepad.Button leftBumper, rightBumper;
    private org.firstinspires.ftc.robotlib.hardware.gamepad.Button leftStickButton, rightStickButton;

    // FLOAT VALUES ADDED
    public float left_stick_x, left_stick_y, right_stick_x, right_stick_y;
    public float left_trigger, right_trigger;


    public RadicalGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
        dpadUp = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.dpad_up);
        dpadDown = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.dpad_down);
        dpadLeft = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.dpad_left);
        dpadRight = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.dpad_right);

        A = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.a);
        B = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.b);
        X = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.x);
        Y = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.y);

        leftBumper = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.left_bumper);
        rightBumper = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.right_bumper);
        leftStickButton = new org.firstinspires.ftc.robotlib.hardware.gamepad.Button(gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType.left_stick_button);
        rightStickButton = new Button(gamepad, ButtonType.right_stick_button);
    }

    public void update() {
        dpad_up = dpadUp.isPressed();
        dpad_down = dpadDown.isPressed();
        dpad_left = dpadLeft.isPressed();
        dpad_right = dpadRight.isPressed();

        a = A.isPressed();
        b = B.isPressed();
        x = X.isPressed();
        y = Y.isPressed();

        left_bumper = leftBumper.isPressed();
        right_bumper = rightBumper.isPressed();
        left_stick_button = leftStickButton.isPressed();
        right_stick_button = rightStickButton.isPressed();

        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;

        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
    }
}
