package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;

@Config
public class Intake extends AbstractSubsystem {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String FRONT_NAME = "intakeFront";
    private static final String REAR_NAME = "intakeRear";

    public static double FRONT_POWER_ON = 1;
    public static double REAR_POWER_ON = 1;

    private DcMotorEx front, rear;
    private double frontPower = 0, rearPower = 0;

    private RingCounter ringCounter;

    public Intake(HardwareMap hardwareMap, RingCounter ringCounter) {
        super("Intake");

        front = hardwareMap.get(DcMotorEx.class, FRONT_NAME);
        rear = hardwareMap.get(DcMotorEx.class, REAR_NAME);
        rear.setDirection(DcMotorSimple.Direction.REVERSE);

        this.ringCounter = ringCounter;
    }

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    @Override
    public void init() {
        turnOff();
    }

    @Override
    public void update() {
        // INTAKE SAFETY FEATURES //
        int cartridge = ringCounter.getNumberOfRingsInCartridge();
        int total = ringCounter.getTotalRings();

        if (total > 3) {
            //turnOnReverse();
        } else if (cartridge == 3 && total == 3) {
            turnOff();
        }
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
        telemetry.put("Front motor power: ", front.getPower());
        telemetry.put("Back motor power: ", rear.getPower());
    }

    @Override
    public void updateLogging() {
        logger.atInfo().log("Front motor power: %f", front.getPower());
        logger.atInfo().log("Back motor power: %f", rear.getPower());
        logger.atInfo().log("Target - front power: %f", frontPower - front.getPower());
        logger.atInfo().log("Target - rear power: %f", rearPower - rear.getPower());
        logger.atInfo().log("Front target: %f", frontPower);
        logger.atInfo().log("Rear target: %f", rearPower);
    }

    // Setters //
    public void turnOn() {
        ringCounter.setReversed(false);
        frontPower = FRONT_POWER_ON;
        rearPower = REAR_POWER_ON;
    }
    public void turnOff() {
        frontPower = 0;
        rearPower = 0;
    }
    public void turnOnReverse() {
        ringCounter.setReversed(true);
        frontPower = -FRONT_POWER_ON;
        rearPower = -REAR_POWER_ON;
    }
}
