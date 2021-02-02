package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

@Config

public class RobotMap {
    // ROBOT INFO //
    // Everything is in inches //
    public static double TIMEOUT = 5000; // In milliseconds

    // The following is relative to the robots center
    public static Vector3D SHOOTER_LOCATION = new Vector3D(0,0,0); // Relative to robot center
    public static Vector3D CAMERA_LOCATION = new Vector3D(0,0,0);
    public static Vector3D ARM_DOWN_LOCATION = new Vector3D(0,0,0);
}
