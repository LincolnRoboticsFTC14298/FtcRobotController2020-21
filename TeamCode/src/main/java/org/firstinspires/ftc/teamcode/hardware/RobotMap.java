package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class RobotMap {
    // ROBOT INFO //
    // Everything is in inches //

    // The following is relative to the robots center
    public static Vector3D SHOOTER_LOCATION = new Vector3D(0,0,0);
    public static Vector2d SHOOTER_LOCATION_2d = new Vector2d(SHOOTER_LOCATION.getX(), SHOOTER_LOCATION.getY());

    public static Vector3D CAMERA_LOCATION = new Vector3D(0,0,0);
    public static Vector2d CAMERA_LOCATION_2d = new Vector2d(CAMERA_LOCATION.getX(), CAMERA_LOCATION.getY());

    public static Vector3D ARM_DOWN_LOCATION = new Vector3D(0,0,0);
    public static Vector2d ARM_DOWN_LOCATION_2d = new Vector2d(ARM_DOWN_LOCATION.getX(), ARM_DOWN_LOCATION.getY());
}
