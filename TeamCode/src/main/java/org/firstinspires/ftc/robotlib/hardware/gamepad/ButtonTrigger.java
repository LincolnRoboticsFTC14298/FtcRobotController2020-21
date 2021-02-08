package org.firstinspires.ftc.robotlib.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;

public class ButtonTrigger extends Button {
    private Gamepad gamepad;
    private double threshold = 0.5;
    private Field field;

    public ButtonTrigger(Gamepad gamepad, ButtonType buttonType) {
        super(gamepad, buttonType);
        this.gamepad = gamepad;
        try {
            this.field = Gamepad.class.getField(buttonType.toString());
        } catch (Exception e) {
            this.field = null;
        }
    }
    public ButtonTrigger(Gamepad gamepad, ButtonType buttonType, double threshold) {
        super(gamepad, buttonType);
        this.gamepad = gamepad;
        this.threshold = threshold;
        try {
            this.field = Gamepad.class.getField(buttonType.toString());
        } catch (Exception e) {
            this.field = null;
        }
    }

    public boolean getRawPressed() {
        try {
            double val = (double) field.get(gamepad);
            if (val > threshold) {
                return true;
            }
        } catch (IllegalAccessException e) {
            //throw new IllegalStateException("Failed to access field " + button.toString() + " after making it accessible", e);
        }
        return false;
    }
}
