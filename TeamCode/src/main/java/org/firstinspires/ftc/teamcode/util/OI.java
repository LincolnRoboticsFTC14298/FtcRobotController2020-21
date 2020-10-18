package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.ComponentOpMode;

public class OI {
    Gamepad gamepad1;
    Gamepad gamepad2;

    public OI(ComponentOpMode opMode) {
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
    }

    public double getLeftStickRadius() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = Math.hypot(x,y);
        return r;
    }
    public double getLeftStickAngle() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double angle = Math.atan2(y,x);
        return angle;
    }
    public double getRightX() {
        double x = gamepad1.right_stick_x;
        return x;
    }

    public boolean autoAim() {
        return gamepad1.a;
    }
}
