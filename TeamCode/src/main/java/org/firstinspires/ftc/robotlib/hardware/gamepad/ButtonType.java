package org.firstinspires.ftc.robotlib.hardware.gamepad;

public enum ButtonType {
    a("a"),
    b("b"),
    x("x"),
    y("y"),
    dpad_up("dpad_up"),
    dpad_down("dpad_down"),
    dpad_left("dpad_left"),
    dpad_right("dpad_right"),
    left_bumper("left_bumper"),
    right_bumper("right_bumper"),
    left_trigger("left_trigger"),
    right_trigger("right_trigger"),
    left_stick_button("left_stick_button"),
    right_stick_button("right_stick_button"),
    back("back"),
    guide("guide"),
    start("start");

    private String name;

    ButtonType(String name) {
        this.name = name;
    }

    public String toString() {
        return name;
    }
}