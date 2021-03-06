package org.firstinspires.ftc.robotlib.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ToggleButton extends Button {
    private boolean toggle;

    public ToggleButton(Gamepad gamepad, ButtonType buttonType) {
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

    public boolean isOn() {
        return get();
    }
    public boolean isOff() {
        return !get();
    }

    public boolean changedToOn() {
        boolean oldToggle = toggle;
        boolean newToggle = get();
        return !oldToggle && newToggle;
    }
    public boolean changedToOff() {
        boolean oldToggle = toggle;
        boolean newToggle = get();
        return oldToggle && !newToggle;
    }
}
