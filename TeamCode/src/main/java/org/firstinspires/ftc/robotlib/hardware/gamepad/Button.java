package org.firstinspires.ftc.robotlib.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Field;

public class Button {
    private Gamepad gamepad;
    private ButtonType buttonType;
    private Field field;


    private boolean state;
    private boolean down; // Down means it has already been pressed


    private boolean debounce;
    private double debouncePeriod;
    private ElapsedTime elapsedTime = new ElapsedTime();

    // possible interface method
//    public static interface IsPressed {
//        boolean isPressed();
//    }
//
//    public static enum Button2 {
//
//        private static interface LateBinding {
//            boolean isPressed(Gamepad gamepad);
//        }
//
//        private LateBinding lateBinding;
//
//        public boolean isPressed(Gamepad gamepad) {
//            lateBinding.isPressed(gamepad);
//        }
//    }



    // Possible Listener method
//    private static interface ButtonListener {
//        void onShortPress();
//        void onLongPress();
//    }
//
//    {
//        MyGamepad myGamepad;
//        myGampad.addListener(Button.A, new ButtonListener(){});
//
//    }

    //



    public Button(Gamepad gamepad, ButtonType buttonType) {
        this.gamepad = gamepad;
        this.buttonType = buttonType;
        try {
            this.field = Gamepad.class.getField(buttonType.toString());
        } catch (Exception e) {
            this.field = null;
        }
        this.debounce = false;
    }
    public Button(Gamepad gamepad, ButtonType buttonType, double debouncePeriod) {
        this.gamepad = gamepad;
        this.buttonType = buttonType;
        try {
            this.field = Gamepad.class.getField(buttonType.toString());
        } catch (Exception e) {
            this.field = null;
        }
        this.debounce = true;
        this.debouncePeriod = debouncePeriod; // Default period
    }

    public ButtonType getButtonType() {
        return buttonType;
    }

    public boolean isPressed() {
        boolean raw = getRawPressed();

        if (debounce) {
            raw = debouncePressed(raw);
        }

        if (raw && down) {
            state = false;
        } else if (raw) {
            down = true;
            state = true;
        } else {
            state = false;
            down = false;
        }

        return state;
    }

    private boolean debouncePressed(boolean pressed) {
        if (pressed && elapsedTime.milliseconds() / 1000.0 > debouncePeriod) {
            elapsedTime.reset();
            return true;
        } else {
            return false;
        }
    }

    private boolean getRawPressed() {
        try {
            return (boolean) field.get(gamepad);
        } catch (IllegalAccessException e) {
            //throw new IllegalStateException("Failed to access field " + button.toString() + " after making it accessible", e);
        }
        return false;
    }
}
