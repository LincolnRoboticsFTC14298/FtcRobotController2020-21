package org.firstinspires.ftc.robotlib.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class MathUtil {
    public static Vector2D poseToVector2D(Pose2d pose) {
        return new Vector2D(pose.getX(), pose.getY());
    }
    public static Vector3D poseToVector3D(Pose2d pose) {
        return new Vector3D(pose.getX(), pose.getY(), 0);
    }

    public static Pose2d vector2DToPose(Vector2D vector, double heading) {
        return new Pose2d(vector.getX(), vector.getY(), heading);
    }

    public static Vector2D vector3DToVector2D(Vector3D vector) {
        return new Vector2D(vector.getX(), vector.getY());
    }
    public static Vector3D vector2DToVector3D(Vector2D vector) {
        return new Vector3D(vector.getX(), vector.getY(),0);
    }

    public static Vector2D vectorFromAngle(double angle) {
        return new Vector2D(Math.cos(angle), Math.sin(angle));
    }
    public static Vector2D vectorFromAngle(double magnitude, double angle) {
        return new Vector2D(Math.cos(angle), Math.sin(angle)).scalarMultiply(magnitude);
    }

    public static Vector2D rotateVector(Vector2D vector, double angle) {
        double x = vector.getX(), y = vector.getY();
        return new Vector2D(x * Math.cos(angle) - y * Math.sin(angle),
                            x * Math.sin(angle) + y * Math.cos(angle));
    }

    public static double angle(Vector2D v1, Vector2D v2) {
        // Angle from v1 to v2
        double angle = Vector2D.angle(v1, v2);
        return angle * Math.signum(v1.normalize().getX() - v2.normalize().getX());
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

    public static boolean withinRange(double number, double min, double max) {
        return number >= min && number <= max;
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
