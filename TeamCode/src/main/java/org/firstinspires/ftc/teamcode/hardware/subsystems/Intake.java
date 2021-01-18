package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotlib.hardware.Subsystem;

@Config
public class Intake extends Subsystem {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String FRONT_NAME = "intakeFront";
    private static final String REAR_NAME = "intakeRear";

    public static double FRONT_POWER_ON = 1;
    public static double REAR_POWER_ON = 1;

    private final DcMotorEx front;
    private final DcMotorEx rear;
    private double frontPower = 0, rearPower = 0;

    public Intake(HardwareMap hardwareMap) {
        super("Intake");

        front = hardwareMap.get(DcMotorEx.class, FRONT_NAME);
        rear = hardwareMap.get(DcMotorEx.class, REAR_NAME);
        rear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init() {
        turnOff();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        turnOff();
    }

    @Override
    public void updateMotorAndServoValues() {
        front.setPower(frontPower);
        rear.setPower(rearPower);
    }

    @Override
    public void updateTelemetry() {
        telemetry.put("front motor power: ", front.getPower());
        telemetry.put("back motor power: ", rear.getPower());
    }

    @Override
    public void updateLogging() {
        logger.atFine().log("front motor power: ", front.getPower());
        logger.atFine().log("back motor power: ", rear.getPower());
        logger.atFiner().log("target - front power: ", frontPower - front.getPower());
        logger.atFiner().log("target - rear power: ", rearPower - rear.getPower());
        logger.atFiner().log("front target: ", frontPower);
        logger.atFiner().log("rear target: ", rearPower);
    }

    public void turnOn() {
        frontPower = FRONT_POWER_ON;
        rearPower = REAR_POWER_ON;
    }
    public void turnOff() {
        frontPower = 0;
        rearPower = 0;
    }
}
