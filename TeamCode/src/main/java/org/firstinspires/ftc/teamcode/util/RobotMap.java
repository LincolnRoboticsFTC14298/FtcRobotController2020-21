package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.opencv.core.Point3;


public class RobotMap {
    // ROBOT INFO //
    // Everything is in inches
    public static final double ROBOT_WIDTH = 18, ROBOT_HEIGHT = 18;
    // The following is relative to the robots center
    public static final Point3 CAMERA_POS = new Point3(0,0,0);

    public static final Point3 ODOMETERY_WHEEL_LEFT_POS = new Point3(0,0,0);
    public static final Point3 ODOMETERY_WHEEL_RIGHT_POS = new Point3(0,0,0);
    public static final Point3 ODOMETERY_WHEEL_CENTER_POS = new Point3(0,0,0);



    // DRIVE CONSTANTS //
    public static final String FRONT_LEFT_NAME = "frontLeft";
    public static final String FRONT_RIGHT_NAME = "frontRight";
    public static final String BACK_LEFT_NAME = "backLeft";
    public static final String BACK_RIGHT_NAME = "backRight";



    // ARM CONSTANTS //
    public static final String CLAW_SERVO_NAME = "claw";
    public static final String ARM_SERVO_NAME = "arm";

    public static final double CLAW_CLOSE_ANGLE = 0;
    public static final double CLAW_OPEN_ANGLE = 1;



    // INTAKE //
    public static final String INTAKE_MOTOR_NAME = "intake";

    public static final double INTAKE_POWER_ON = 1;



    // AUTONOMOUS //


}
