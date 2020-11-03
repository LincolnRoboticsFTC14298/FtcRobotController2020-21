package org.firstinspires.ftc.teamcode.util;

public class MathUtil {
    public static double angleWrapRadians(double angle) {
        // Makes angle between -180 180
        double x = (angle + Math.PI) % 2*Math.PI;
        if (x < 0)
            x += 2*Math.PI;
        return x - Math.PI;
    }
    public static double angleWrapDegrees(double angle) {
        // Makes angle between -180 180
        double x = (angle + 180) % 360;
        if (x < 0)
            x += 360;
        return x - 180;
    }
}
