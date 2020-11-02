package org.firstinspires.ftc.teamcode.hardware.util.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ToggleButton {
    private Button button;
    private boolean toggle;

    public ToggleButton(Gamepad gamepad, ButtonType buttonType) {
        button = new Button(gamepad, buttonType);
    }
    public ToggleButton(Gamepad gamepad, ButtonType buttonType, double debouncePeriod) {
        button = new Button(gamepad, buttonType, debouncePeriod);
    }

    private void get() {
        if (button.isPressed()) {
            toggle = !toggle;
        }
    }

    public boolean on() {
        return toggle;
    }
    public boolean off() {
        return !toggle;
    }
}
