package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import robotlib.hardware.Subsystem;
import robotlib.util.MathUtil;

@Config
public class Shooter implements Subsystem {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    //private static final FluentLogger logger = FluentLogger.forEnclosingClass();
    //private static final Telemetry telemetry = new Telemetry("Shooter");

    private static final String SHOOTER_MOTOR1_NAME = "shooter1", SHOOTER_MOTOR2_NAME = "shooter2";
    private static final String FLAP_NAME = "flap", LAUNCH_FLAP_NAME = "launchFlap";

    public static double SHOOTER_MIN_ERROR = 0.1;
    public static double SHOOTER_DEFAULT_POWER = .75;

    // In radians
    public static double FLAP_MIN_ERROR = .03;
    public static double FLAP_MIN_ANGLE = 0;
    public static double FLAP_MAX_ANGLE = 0;

    public static double LAUNCH_FLAP_RETRACTED_MIN_ERROR = 0.1;
    public static double LAUNCH_FLAP_EXTENDED_MIN_ERROR = 0.2;
    public static double LAUNCH_FLAP_RETRACTED_POS = 0;
    public static double LAUNCH_FLAP_EXTENDED_POS = 1;


    private DcMotor shooterMotor1, shooterMotor2;
    private Servo flapServo, launchFlapServo;

    private double shooterMotor1Power, shooterMotor2Power;
    private double flapAngle, launchFlapPos;
    private double targetAngle = 0;

    private PositionProvider positionProvider;

    public Shooter(HardwareMap hardwareMap, PositionProvider positionProvider) {
        // Initialize motors and servos //
        shooterMotor1 = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR1_NAME);
        shooterMotor2 = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR2_NAME);

        flapServo = hardwareMap.get(Servo.class, FLAP_NAME);
        launchFlapServo = hardwareMap.get(Servo.class, LAUNCH_FLAP_NAME);

        this.positionProvider = positionProvider;

        // Reverse direction
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        setFlapAngle(Math.toRadians(45));
        retractLaunchFlap();
    }

    @Override
    public void update() {
        setShooterPower(SHOOTER_DEFAULT_POWER);
    }

    @Override
    public void stop() {
        setFlapAngle(Math.toRadians(45));
        retractLaunchFlap();
    }

    @Override
    public void updateMotorsAndServos() {
        shooterMotor1.setPower(shooterMotor1Power);
        shooterMotor2.setPower(shooterMotor2Power);

        flapServo.setPosition(angleToPos(flapAngle));
        launchFlapServo.setPosition(launchFlapPos);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Shooter power: ", shooterMotor1.getPower());
        packet.addLine("Flap Angle: " +  Math.toDegrees(targetAngle) + " target vs " +
                Math.toDegrees(posToAngle(flapServo.getPosition())) + " actual");
        dashboard.sendTelemetryPacket(packet);
    }


    public boolean doneAiming() {
        return Math.abs(posToAngle(flapServo.getPosition()) - flapAngle) < FLAP_MIN_ERROR;
    }
    public void aimAsync() {
        setFlapAngle(positionProvider.getTargetLaunchAngle());
    }

    public boolean readyToLaunch() {
        return doneAiming() &&
                MathUtil.withinRange(positionProvider.getTargetLaunchAngle(), FLAP_MIN_ANGLE, FLAP_MAX_ANGLE) &&
                Math.abs(shooterMotor1.getPower() - shooterMotor1Power) < SHOOTER_MIN_ERROR &&
                Math.abs(shooterMotor2.getPower() - shooterMotor2Power) < SHOOTER_MIN_ERROR;
    }

    public boolean isExtended() {
        return Math.abs(launchFlapServo.getPosition() - LAUNCH_FLAP_EXTENDED_POS) < LAUNCH_FLAP_EXTENDED_MIN_ERROR;
    }
    public boolean isRetracted() {
        return Math.abs(launchFlapServo.getPosition() - LAUNCH_FLAP_RETRACTED_POS) < LAUNCH_FLAP_RETRACTED_MIN_ERROR;
    }

    public void launch() {
        extendLaunchFlap();
        while (!isExtended()) {
            updateMotorsAndServos();
        }
        retractLaunchFlap();
        while (!isRetracted()) {
            updateMotorsAndServos();
        }
    }


    public void turnOnShooterMotor() {
        setShooterPower(SHOOTER_DEFAULT_POWER);
    }
    public void turnOffShooterMotor() {
        setShooterPower(0);
    }

    // Angle is in radians
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
