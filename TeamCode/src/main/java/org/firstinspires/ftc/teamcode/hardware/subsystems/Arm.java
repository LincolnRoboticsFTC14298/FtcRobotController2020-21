package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;

@Config
public class Arm extends AbstractSubsystem {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    public static final String CLAW_SERVO_NAME = "claw";
    public static final String ARM_MOTOR_NAME = "arm";

    // Position from 0 to 1
    public static double CLAW_OPEN_POSITION = 0.40;
    public static double CLAW_CLOSE_POSITION = 0.58;

    public static double ARM_DEFAULT_ANGLE = Math.toRadians(126);
    public static double ARM_LIFT_ANGLE = ARM_DEFAULT_ANGLE; // to travel
    public static double ARM_MIDDLE_ANGLE = Math.toRadians(57); // to drop off
    public static double ARM_LOWER_ANGLE = Math.PI/3; // to pick up

    public static double speed = .5;

    public static double GEAR_RATIO = 1; // output revs / input revs

    public static PIDFCoefficients POS_PIDF = new PIDFCoefficients(0,0,0,0);

    public static double TICKS_PER_REV = 1425.1;

    private VoltageSensor batteryVoltageSensor;

    private DcMotorEx armMotor;
    private Servo clawServo;
    private double armAngle, clawPosition;

    public Arm(HardwareMap hardwareMap) {
        super("Arm");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        clawServo = hardwareMap.get(Servo.class, CLAW_SERVO_NAME);

        armMotor = hardwareMap.get(DcMotorEx.class, ARM_MOTOR_NAME);
        armMotor.setDirection(DcMotorEx.Direction.REVERSE);

        armMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, POS_PIDF);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init() {
        closeClaw(); // At the beginning of the round, the claw is closed with the wobble
        defaultPos();
    }


    @Override
    public void start() {
        closeClaw();
        //lower();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        closeClaw();
        lower();
    }

    @Override
    public void updateMotorAndServoValues() {
        clawServo.setPosition(clawPosition);

        if (armMotor.isBusy()) {
            armMotor.setPower(speed);
        }
        else {
            armMotor.setPower(0);
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void updateTelemetry() {
        telemetry.put("Claw position", clawServo.getPosition());
        telemetry.put("Arm angle", getArmAngle());
    }

    @Override
    public void updateLogging() {
        logger.atInfo().log("Claw position: %f", clawServo.getPosition());
        logger.atInfo().log("Arm angle: %f", getArmAngle());
        logger.atInfo().log("Target - claw position: %f", clawPosition - clawServo.getPosition());
        logger.atInfo().log("Target - arm angle: %f", armAngle - getArmAngle());
        logger.atInfo().log("Claw target: %f", clawPosition);
        logger.atInfo().log("Arm target: %f", armAngle);
    }

    // Commands //
    public void openClaw() {
        setClawPosition(CLAW_OPEN_POSITION);
    }
    public void closeClaw() {
        setClawPosition(CLAW_CLOSE_POSITION);
    }

    public void lift() {
        setArmAngle(ARM_LIFT_ANGLE);
    }
    public void defaultPos() {
        setArmAngle(ARM_DEFAULT_ANGLE);
    }
    public void middle() {
        setArmAngle(ARM_MIDDLE_ANGLE);
    }
    public void lower() {
        setArmAngle(ARM_LOWER_ANGLE);
    }

    public void resetDefaultAngle() {
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Setters //
    public void setArmAngle(double angle) {
        armAngle = angle;
        updateArmAngle(armAngle);
    }
    private void updateArmAngle(double angle) {
        angle -= ARM_DEFAULT_ANGLE;
        int ticks = (int) (angle / (2 * Math.PI * GEAR_RATIO) * TICKS_PER_REV);
        armMotor.setTargetPosition(ticks);
        logger.atInfo().log("Ticks: %d", ticks);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setClawPosition(double position) {
        this.clawPosition = position;
    }

    // Getters //
    public double getTargetArmAngle() {
        return armAngle;
    }

    public double getArmAngle() {
        return 2 * Math.PI * GEAR_RATIO * armMotor.getCurrentPosition() / TICKS_PER_REV + ARM_DEFAULT_ANGLE;
    }

    private void setMode(DcMotor.RunMode mode) {
        if (!armMotor.getMode().equals(mode)) {
            armMotor.setMode(mode);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        armMotor.setPIDFCoefficients(runMode, compensatedCoefficients);
    }
}
