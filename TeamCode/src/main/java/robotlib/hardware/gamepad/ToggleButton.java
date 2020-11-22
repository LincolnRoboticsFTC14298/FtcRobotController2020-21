package robotlib.hardware.gamepad;

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

    public boolean on() {
        get();
        return toggle;
    }
    public boolean off() {
        get();
        return !toggle;
    }
}
