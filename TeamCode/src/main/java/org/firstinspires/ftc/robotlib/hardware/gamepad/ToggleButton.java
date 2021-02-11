package org.firstinspires.ftc.robotlib.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ToggleButton extends Button {
    private boolean toggle;

    public ToggleButton(Gamepad gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType buttonType) {
        super(gamepad, buttonType);
    }
    public ToggleButton(Gamepad gamepad, ButtonType buttonType, double debouncePeriod) {
        super(gamepad, buttonType, debouncePeriod);
    }

    public boolean get() {
        if (isPressed()) {
            toggle = !toggle;
        }
        return toggle;
    }

    public boolean on() {
        return get();
    }
    public boolean off() {
        return !get();
    }

    public boolean changedToOn() {
        return isPressed() && on();
    }
    public boolean changedToOff() {
        return isPressed() && off();
    }
}
