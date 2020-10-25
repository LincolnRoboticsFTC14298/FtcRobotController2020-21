package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;


public class RobotMap {
    // ROBOT INFO //
    // Everything is in inches
    public static final double ROBOT_WIDTH = 18, ROBOT_HEIGHT = 18;
    // The following is relative to the robots center


    public static final Vector3D ODOMETERY_WHEEL_LEFT_POS = new Vector3D(0,0,0);
    public static final Vector3D ODOMETERY_WHEEL_CENTER_POS = new Vector3D(0,0,0);
    public static final Vector3D ODOMETERY_WHEEL_RIGHT_POS = new Vector3D(0,0,0);


    public static final double TIMEOUT = 5000; // In milliseconds

    // OI CONSTANTS //
    public static final double DEBOUNCER_PERIOD = 0.5;

    // AUTONOMOUS //



    // ARM CONSTANTS //
    public static final String CLAW_SERVO_NAME = "claw";
    public static final String ARM_SERVO_NAME = "arm";

    // Position from 0 to 1
    public static final double CLAW_OPEN_POSITION = 1;
    public static final double CLAW_CLOSE_POSITION = 0;

    public static final double ARM_LIFT_POSITION = 1;
    public static final double ARM_DEFAULT_POSITION = .5;
    public static final double ARM_LOWER_POSITION = 0;



    // DRIVE CONSTANTS //
    public static final String FRONT_LEFT_NAME = "frontLeft";
    public static final String FRONT_RIGHT_NAME = "frontRight";
    public static final String BACK_LEFT_NAME = "backLeft";
    public static final String BACK_RIGHT_NAME = "backRight";

    public static final double DRIVER_TARGET_ANGLE_MIN_ERROR = 1; // In degrees



    // INTAKE CONSTANTS //
    public static final String INTAKE_MOTOR1_NAME = "intakeFront";
    public static final String INTAKE_MOTOR2_NAME = "intakeBack";

    public static final double INTAKE_MOTOR1_POWER_ON = 1;
    public static final double INTAKE_MOTOR2_POWER_ON = 1;



    // POSITION LOCALIZER CONSTANTS //



    // ROAD RUNNER CONSTANTS //



    // Shooter CONSTANTS //
    public static final Vector3D SHOOTER_LOCATION = new Vector3D(0,0,0); // Relative to robot center

    public static final String MOTOR1_NAME = "motor1", MOTOR2_NAME = "motor2", LOAD_MOTOR_NAME = "loadMotor";
    public static final String FLAP_NAME = "flap";

    // In radians
    public static final double FLAP_MIN_ERROR = .03;
    public static final double FLAP_MIN_ANGLE = 0;
    public static final double FLAP_MAX_ANGLE = 0;

    public static final double SHOOTER_MIN_ERROR = 0.1;
    public static final double SHOOTER_DEFAULT_POWER = .75;

    public static final double LOAD_MOTOR_POWER = 1;
    public static final double LOAD_MOTOR_DELAY = .5;



    // VISION CONSTANTS //
    public static final Vector3D CAMERA_LOCATION = new Vector3D(0,0,0);
}
