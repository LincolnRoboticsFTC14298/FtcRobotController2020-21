package org.firstinspires.ftc.robotlib.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;

public class ButtonTrigger extends Button {
    private Gamepad gamepad;
    private double threshold = 0.5;

    public ButtonTrigger(Gamepad gamepad, org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType buttonType) {
        super(gamepad, buttonType);
        this.gamepad = gamepad;
    }
    public ButtonTrigger(Gamepad gamepad, ButtonType buttonType, double threshold) {
        super(gamepad, buttonType);
        this.gamepad = gamepad;
        this.threshold = threshold;
    }

    public boolean getRawPressed() {
        try {
            Field f = Gamepad.class.getField(getButtonType().toString());
            double val = (double) f.get(gamepad);
            if (val > threshold) {
                return true;
            }
        } catch (NoSuchFieldException e) {
            //throw new IllegalStateException("Bad field name: " + button.toString(), e);
        } catch (IllegalAccessException e) {
            //throw new IllegalStateException("Failed to access field " + button.toString() + " after making it accessible", e);
        }
        return false;
    }
}
