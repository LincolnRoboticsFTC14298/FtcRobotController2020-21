package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;
import org.firstinspires.ftc.robotlib.util.MathUtil;

@Config
public class Shooter extends AbstractSubsystem {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String SHOOTER_MOTOR_NAME = "shooter";
    private static final String FLAP_NAME = "flap", LAUNCH_FLAP_NAME = "launchFlap";

    public static double SHOOTER_MIN_ERROR = 0.1;
    public static double SHOOTER_DEFAULT_POWER = .75;

    // In radians
    public static double FLAP_MIN_ERROR = .03;
    public static double FLAP_MIN_ANGLE = 0;
    public static double FLAP_MAX_ANGLE = Math.PI/2.0;

    public static double LAUNCH_FLAP_RETRACTED_MIN_ERROR = 0.1;
    public static double LAUNCH_FLAP_EXTENDED_MIN_ERROR = 0.2;
    public static double LAUNCH_FLAP_RETRACTED_POS = 0;
    public static double LAUNCH_FLAP_EXTENDED_POS = 1;

    private DcMotor shooterMotor;
    private Servo flapServo, launchFlapServo;

    private double shooterMotorPower;
    private double flapAngle, launchFlapPos;
    private double targetAngle = 0;

    private Localizer localizer;

    public enum LaunchStatus {
        EXTENDING,
        RETRACTING,
        RETRACTED
    }

    public enum AimingMode {
        AIMING,
        IDLE
    }

    private LaunchStatus launchStatus = LaunchStatus.RETRACTED;
    private AimingMode aimingMode = AimingMode.IDLE;

    public Shooter(HardwareMap hardwareMap, Localizer localizer) {
        super("Shooter");
        // Initialize motors and servos //
        shooterMotor = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR_NAME);

        flapServo = hardwareMap.get(Servo.class, FLAP_NAME);
        launchFlapServo = hardwareMap.get(Servo.class, LAUNCH_FLAP_NAME);

        // Reverse direction
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.localizer = localizer;
    }

    @Override
    public void init() {
        stopAiming();
    }

    @Override
    public void start() {
        setFlapAngle(Math.toRadians(45));
        retractLaunchFlap();
    }

    @Override
    public void update() {
        setShooterPower(SHOOTER_DEFAULT_POWER);
        updateAiming();
        updateLaunching();
    }

    @Override
    public void stop() {
        stopAiming();
        setFlapAngle(Math.toRadians(45));
        retractLaunchFlap();
    }

    @Override
    public void updateMotorAndServoValues() {
        shooterMotor.setPower(shooterMotorPower);

        flapServo.setPosition(angleToPos(flapAngle));
        launchFlapServo.setPosition(launchFlapPos);
    }

    @Override
    public void updateTelemetry() {
        telemetry.put("Aiming mode: ", aimingMode);
        telemetry.put("Launch status: ", launchStatus);
        telemetry.put("Shooter power: ", shooterMotor.getPower());
        telemetry.put("Flap angle: ", Math.toDegrees(posToAngle(flapServo.getPosition())));
    }

    @Override
    public void updateLogging() {
        logger.atInfo().log("Aiming mode: %s", aimingMode);
        logger.atInfo().log("Launch status: %s", launchStatus);
        logger.atInfo().log("Shooter power: %f", shooterMotor.getPower());
        logger.atInfo().log("Flap angle: %f", Math.toDegrees(posToAngle(flapServo.getPosition())));

        logger.atInfo().log("Target - shoot power: %f", shooterMotorPower - shooterMotor.getPower());
        logger.atInfo().log("Target - flap angle: %f", Math.toDegrees(targetAngle) - Math.toDegrees(posToAngle(flapServo.getPosition())));

        logger.atInfo().log("Ready to launch: %b", readyToLaunch());
        logger.atInfo().log("Done aiming: %b", doneAiming());
        logger.atInfo().log("Is extended: %b", isExtended());
        logger.atInfo().log("Is retracted: %b", isRetracted());
    }


    // Aiming //
    public boolean doneAiming() {
        return Math.abs(posToAngle(flapServo.getPosition()) - flapAngle) < FLAP_MIN_ERROR;
    }
    public void aimAsync() {
        aimingMode = AimingMode.AIMING;
    }
    public void stopAiming() {
        aimingMode = AimingMode.IDLE;
    }
    public void updateAiming() {
        switch (aimingMode) {
            case IDLE:
                break;
            case AIMING:
                setFlapAngle(localizer.getTargetLaunchAngle());
                break;
        }
    }

    // Launching //
    public boolean readyToLaunch() {
        return doneAiming() &&
                MathUtil.inRange(localizer.getTargetLaunchAngle(), FLAP_MIN_ANGLE, FLAP_MAX_ANGLE) &&
                MathUtil.differenceWithinError(shooterMotor.getPower(), shooterMotorPower, SHOOTER_MIN_ERROR) &&
                isRetractedStatus(); // Could use isRetracted();
    }
    public void launchAsync() {
        launchStatus = LaunchStatus.EXTENDING;
    }
    public void launch() {
        extendLaunchFlap();
        while (!isExtended() && !Thread.currentThread().isInterrupted()) {
            updateMotorAndServoValues();
        }
        retractLaunchFlap();
    }
    public void updateLaunching() {
        switch (launchStatus) {
            case EXTENDING:
                if (isExtended()) {
                    launchStatus = LaunchStatus.RETRACTING;
                    retractLaunchFlap();
                }
                break;
            case RETRACTING:
                if (isRetracted()) {
                    launchStatus = LaunchStatus.RETRACTED;
                }
                break;
        }
    }


    // Getters and Setters //
    public LaunchStatus getLaunchStatus() {
        return launchStatus;
    }
    public boolean isExtended() {
        return MathUtil.differenceWithinError(launchFlapServo.getPosition(), LAUNCH_FLAP_EXTENDED_POS, LAUNCH_FLAP_EXTENDED_MIN_ERROR);
    }
    public boolean isRetracted() {
        return MathUtil.differenceWithinError(launchFlapServo.getPosition(), LAUNCH_FLAP_RETRACTED_POS, LAUNCH_FLAP_RETRACTED_MIN_ERROR);
    }
    public boolean isRetractedStatus() {
        return launchStatus == LaunchStatus.RETRACTED;
    }

    public void turnOnShooterMotor() {
        setShooterPower(SHOOTER_DEFAULT_POWER);
    }
    public void turnOffShooterMotor() {
        setShooterPower(0);
    }

    public double getTargetAngle() {
        return flapAngle;
    }
    public void setFlapAngle(double angle) {
        flapAngle = Range.clip(angle, FLAP_MIN_ANGLE, FLAP_MAX_ANGLE);
    }

    public void retractLaunchFlap() {
        launchFlapPos = LAUNCH_FLAP_RETRACTED_POS;
    }
    public void extendLaunchFlap() {
        launchFlapPos = LAUNCH_FLAP_EXTENDED_POS;
    }

    private void setShooterPower(double power) {
        shooterMotorPower = Range.clip(power, -1, 1);
    }

    private double posToAngle(double pos) {
        // Scale from 0 = minAngle to 1 = maxAngle
        return Range.scale(pos, 0,1, FLAP_MIN_ANGLE, FLAP_MAX_ANGLE);
    }
    private double angleToPos(double angle) {
        // Scale from minAngle = 0 to maxAngle = 1
        return Range.scale(angle, FLAP_MIN_ANGLE, FLAP_MAX_ANGLE, 0, 1);
    }
}