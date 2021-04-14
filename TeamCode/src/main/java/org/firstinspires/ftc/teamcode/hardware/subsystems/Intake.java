package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;

@Config
public class Intake extends AbstractSubsystem {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String INTAKE_MOTOR_NAME = "intake";

    public static double MOTOR_POWER_ON = 1;

    private DcMotorEx intakeMotor;
    private double intakePower = 0;

    private RingCounter ringCounter;

    public Intake(HardwareMap hardwareMap, RingCounter ringCounter) {
        super("Intake");

        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        //intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        if (ringCounter != null) {
            int cartridge = ringCounter.getNumberOfRingsInCartridge();
            int total = ringCounter.getTotalRings();

            if (total > 3) {
                turnOnReverse();
            } else if (total > cartridge) {
                turnOn();
            } else if (ringCounter.allRingsInCartridge()) {
                turnOff();
            }
        }
    }

    @Override
    public void stop() {
        turnOff();
    }

    @Override
    public void updateMotorAndServoValues() {
        intakeMotor.setPower(intakePower);
    }

    @Override
    public void updateTelemetry() {
        telemetry.put("Intake motor power: ", intakeMotor.getPower());
    }

    @Override
    public void updateLogging() {
        logger.atInfo().log("Intake motor power: %f", intakeMotor.getPower());
        logger.atInfo().log("Intake target: %f", intakePower);
        logger.atInfo().log("Target - intake power: %f", intakePower - intakeMotor.getPower());
    }

    // Setters //
    public void turnOn() {
        if (ringCounter != null) ringCounter.setReversed(false);
        intakePower = MOTOR_POWER_ON;
    }
    public void turnOff() {
        intakePower = 0;
    }
    public void turnOnReverse() {
        if (ringCounter != null) ringCounter.setReversed(true);
        intakePower = -MOTOR_POWER_ON;
    }
}
