package org.firstinspires.ftc.robotlib.hardware.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Field;

public class Button {
    private Gamepad gamepad;
    private ButtonType buttonType;


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
        this.debounce = false;
    }
    public Button(Gamepad gamepad, ButtonType buttonType, double debouncePeriod) {
        this.gamepad = gamepad;
        this.buttonType = buttonType;
        this.debounce = true;
        this.debouncePeriod = debouncePeriod; // Default period
    }

    public ButtonType getButtonType() {
        return buttonType;
    }

    public boolean isPressed() {
        // TODO: check to see if stickyness should be a part of debounce
        boolean raw = getRaw();

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
            Field f = Gamepad.class.getField(buttonType.toString());
            return (boolean) f.get(gamepad);
        } catch (NoSuchFieldException e) {
            //throw new IllegalStateException("Bad field name: " + button.toString(), e);
        } catch (IllegalAccessException e) {
            //throw new IllegalStateException("Failed to access field " + button.toString() + " after making it accessible", e);
        }
        return false;
    }
    private boolean getRaw() {
        switch(buttonType) {
            case a:
                return gamepad.a;
            case b:
                return gamepad.b;
            case x:
                return gamepad.x;
            case y:
                return gamepad.y;
            case dpad_up:
                return gamepad.dpad_up;
            case dpad_down:
                return gamepad.dpad_down;
            case dpad_right:
                return gamepad.dpad_right;
            case dpad_left:
                return gamepad.dpad_left;
            case right_bumper:
                return gamepad.right_bumper;
            case left_bumper:
                return gamepad.left_bumper;
            case right_stick_button:
                return gamepad.right_stick_button;
            case left_stick_button:
                return gamepad.left_stick_button;
            case start:
                return gamepad.back;
            case guide:
                return gamepad.guide;
            case back:
                return gamepad.start;
            default:
                return false;
        }
    }
}