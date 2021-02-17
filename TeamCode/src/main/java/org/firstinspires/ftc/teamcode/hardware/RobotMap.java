package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotlib.util.MathUtil;

public class RobotMap {
    // ROBOT INFO //
    // Everything is in inches //

    // The following is relative to the robots center
    public static Vector3D SHOOTER_LOCATION = new Vector3D(0,0,0);
    public static Vector2d SHOOTER_LOCATION_2d = MathUtil.vector3DToVector2d(SHOOTER_LOCATION);

    public static double CAMERA_PITCH = 0; // RADIANS, positive if pointing down
    public static double CAMERA_YAW = 0;
    public static Vector3D CAMERA_LOCATION = new Vector3D(0,0,0);
    public static Vector2d CAMERA_LOCATION_2d = MathUtil.vector3DToVector2d(CAMERA_LOCATION);

    public static Vector3D ARM_DOWN_LOCATION = new Vector3D(0,0,0);
    public static Vector2d ARM_DOWN_LOCATION_2d = MathUtil.vector3DToVector2d(ARM_DOWN_LOCATION);
}
