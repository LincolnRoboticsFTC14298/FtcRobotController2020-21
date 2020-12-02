package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import robotlib.hardware.Subsystem;

@Config
public class Shooter implements Subsystem {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    //private static final FluentLogger logger = FluentLogger.forEnclosingClass();
    //private static final Telemetry telemetry = new Telemetry("Shooter");

    private static final String MOTOR1_NAME = "motor1", MOTOR2_NAME = "motor2";
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


    private static final double LAMBDA = 1; // = g / (2*v0*v0) optimal lambda will be found and used

    private DcMotor motor1, motor2;
    private Servo flap, launchFlap;

    private double motor1Power, motor2Power;
    private double flapAngle, launchFlapPos;
    private double targetAngle = 0;

//    private Target target = Target.HIGH_GOAL;
//    private Alliance alliance = Alliance.BLUE;

    private PositionProvider positionProvider;

    private double shootScheduler = 0;
    private boolean loading = false;

    public Shooter(HardwareMap hardwareMap, PositionProvider positionProvider) {
        // Initialize motors and servos //
        motor1 = hardwareMap.get(DcMotor.class, MOTOR1_NAME);
        motor2 = hardwareMap.get(DcMotor.class, MOTOR2_NAME);

        flap = hardwareMap.get(Servo.class, FLAP_NAME);
        launchFlap = hardwareMap.get(Servo.class, LAUNCH_FLAP_NAME);

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


    public boolean doneAiming() {
        return Math.abs(posToAngle(flap.getPosition()) - flapAngle) < FLAP_MIN_ERROR;
    }
    public void aimAsync() {
        Vector3D targetRelativePos = positionProvider.getTargetRelativeLocation();
        double dist = Math.hypot(targetRelativePos.getY(), targetRelativePos.getX());
        double height = targetRelativePos.getZ();

        // Math stuff
        double k = 1.0/(2*dist*LAMBDA);
        double det = 1 - 4*(height + LAMBDA*dist*dist)*LAMBDA;
        targetAngle = Math.atan(k - k * Math.sqrt(det));

        setFlapAngle(targetAngle);
    }
    public void aim() {
        aimAsync();
        while (!doneAiming()) {
            update();
            updateMotorsAndServos();
        }
    }

    public boolean readyToLaunch() {
        return doneAiming() &&
                Math.abs(motor1.getPower() - motor1Power) < SHOOTER_MIN_ERROR &&
                Math.abs(motor2.getPower() - motor2Power) < SHOOTER_MIN_ERROR;
    }

    public void launch() {
        extendLaunchFlap();
        while (Math.abs(launchFlap.getPosition() - launchFlapPos) < LAUNCH_FLAP_EXTENDED_MIN_ERROR) {
            updateMotorsAndServos();
        }
        retractLaunchFlap();
        while (Math.abs(launchFlap.getPosition() - launchFlapPos) < LAUNCH_FLAP_RETRACTED_MIN_ERROR) {
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
        flapAngle = angle;
    }

    public void retractLaunchFlap() {
        launchFlapPos = LAUNCH_FLAP_RETRACTED_POS;
    }
    public void extendLaunchFlap() {
        launchFlapPos = LAUNCH_FLAP_EXTENDED_POS;
    }

    private void setShooterPower(double power) {
        // TODO: Clip powers to make sure it's between -1 and 1
        double p = Range.clip(power, -1, 1);
        motor1Power = motor2Power = p;
    }

    private double posToAngle(double pos) {
        // Scale from 0 = minAngle to 1 = maxAngle
        return Range.scale(pos, 0,1, FLAP_MIN_ANGLE, FLAP_MAX_ANGLE);
    }
    private double angleToPos(double angle) {
        // Scale from minAngle = 0 to maxAngle = 1
        return Range.scale(angle, FLAP_MIN_ANGLE, FLAP_MAX_ANGLE, 0, 1);
    }

    @Override
    public void updateMotorsAndServos() {
        motor1.setPower(motor1Power);
        motor2.setPower(motor2Power);

        flap.setPosition(angleToPos(flapAngle));
        launchFlap.setPosition(launchFlapPos);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Shooter power: ", motor1.getPower());
        packet.addLine("Flap Angle: " +  Math.toDegrees(targetAngle) + " target vs " +
                Math.toDegrees(posToAngle(flap.getPosition())) + " actual");
        dashboard.sendTelemetryPacket(packet);
    }
}
