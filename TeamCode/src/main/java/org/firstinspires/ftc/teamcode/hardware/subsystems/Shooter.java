package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotlib.hardware.Subsystem;
import org.firstinspires.ftc.robotlib.util.MathUtil;

@Config
public class Shooter extends Subsystem {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String SHOOTER_MOTOR1_NAME = "shooter1", SHOOTER_MOTOR2_NAME = "shooter2";
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


    private DcMotor shooterMotor1, shooterMotor2;
    private Servo flapServo, launchFlapServo;

    private double shooterMotor1Power, shooterMotor2Power;
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
        shooterMotor1 = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR1_NAME);
        shooterMotor2 = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR2_NAME);

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
        shooterMotor1.setPower(shooterMotor1Power);
        shooterMotor2.setPower(shooterMotor2Power);

        flapServo.setPosition(angleToPos(flapAngle));
        launchFlapServo.setPosition(launchFlapPos);
    }

    @Override
    public void updateTelemetry() {
        telemetry.put("aiming mode: ", aimingMode);
        telemetry.put("launch status: ", launchStatus);
        telemetry.put("shooter power: ", shooterMotor1.getPower());
        telemetry.put("flap angle: ", Math.toDegrees(posToAngle(flapServo.getPosition())));
    }

    @Override
    public void updateLogging() {
        logger.atFine().log("aiming mode: ", aimingMode);
        logger.atFine().log("launch status: ", launchStatus);
        logger.atFine().log("shooter power: ", shooterMotor1.getPower());
        logger.atFine().log("flap angle: ", Math.toDegrees(posToAngle(flapServo.getPosition())));

        logger.atFiner().log("target - shoot power: ", shooterMotor1Power - shooterMotor1.getPower());
        logger.atFiner().log("target - flap angle: ", Math.toDegrees(targetAngle) - Math.toDegrees(posToAngle(flapServo.getPosition())));

        logger.atFinest().log("ready to launch: ", readyToLaunch());
        logger.atFinest().log("done aiming: ", doneAiming());
        logger.atFinest().log("is extended: ", isExtended());
        logger.atFinest().log("is retracted: ", isRetracted());
    }


    // Aiming //
    public boolean doneAiming() {
        return Math.abs(posToAngle(flapServo.getPosition()) - flapAngle) < FLAP_MIN_ERROR;
    }
    public void aimAsync() {
        // TODO: Make update every update()
        aimingMode = AimingMode.AIMING;
        setFlapAngle(localizer.getTargetLaunchAngle());
    }
    public void stopAiming() {
        aimingMode = AimingMode.IDLE;
    }
    public void updateAiming() {
        switch (aimingMode) {
            case AIMING:
                aimAsync();
                break;
        }
    }

    // Launching //
    public boolean readyToLaunch() {
        return doneAiming() &&
                MathUtil.withinRange(localizer.getTargetLaunchAngle(), FLAP_MIN_ANGLE, FLAP_MAX_ANGLE) &&
                Math.abs(shooterMotor1.getPower() - shooterMotor1Power) < SHOOTER_MIN_ERROR &&
                Math.abs(shooterMotor2.getPower() - shooterMotor2Power) < SHOOTER_MIN_ERROR &&
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
        return Math.abs(launchFlapServo.getPosition() - LAUNCH_FLAP_EXTENDED_POS) < LAUNCH_FLAP_EXTENDED_MIN_ERROR;
    }
    public boolean isRetracted() {
        return Math.abs(launchFlapServo.getPosition() - LAUNCH_FLAP_RETRACTED_POS) < LAUNCH_FLAP_RETRACTED_MIN_ERROR;
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
        power = Range.clip(power, -1, 1);
        shooterMotor1Power = shooterMotor2Power = power;
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