package org.firstinspires.ftc.robotlib.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class MathUtil {
    public static Vector3D poseToVector3D(Pose2d pose) {
        return new Vector3D(pose.getX(), pose.getY(), 0);
    }

    public static double inchesToMeters(double inches) {
        return .0254*inches;
    }
    public static Vector2D inchesToMeters(Vector2D inches) {
        return new Vector2D(inchesToMeters(inches.getX()),
                inchesToMeters(inches.getY()));
    }
    public static Vector3D inchesToMeters(Vector3D inches) {
        return new Vector3D(inchesToMeters(inches.getX()),
                inchesToMeters(inches.getY()),
                inchesToMeters(inches.getZ()));
    }


    public static double metersToInches(double meters) {
        return 39.37007874*meters;
    }
    public static Vector2D metersToInches(Vector2D meters) {
        return new Vector2D(inchesToMeters(meters.getX()),
                inchesToMeters(meters.getY()));
    }
    public static Vector3D metersToInches(Vector3D meters) {
        return new Vector3D(inchesToMeters(meters.getX()),
                inchesToMeters(meters.getY()),
                inchesToMeters(meters.getZ()));
    }

    public static boolean inRange(double number, double min, double max) {
        return number >= min && number <= max;
    }

    public static boolean differenceWithinError(double current, double target, double acceptableError) {
        return Math.abs(current - target) < acceptableError;
    }

    public static double angleWrapRadians(double angle) {
        // Makes angle between (-PI, PI)
        double x = (angle + Math.PI) % 2*Math.PI;
        if (x < 0)
            x += 2*Math.PI;
        return x - Math.PI;
    }
    public static double angleWrapDegrees(double angle) {
        // Makes angle between (-180, 180)
        double x = (angle + 180) % 360;
        if (x < 0)
            x += 360;
        return x - 180;
    }
    public static double squareError(double value, double target) {
        double diff = target - value;
        return diff * diff;
    }
}
