package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.Field.Alliance;
import org.firstinspires.ftc.teamcode.util.Field.Target;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.SHOOTER_LOCATION;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.TIMEOUT;

@Config
public class Shooter implements Subsystem {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet;
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String MOTOR1_NAME = "motor1", MOTOR2_NAME = "motor2", LOAD_MOTOR_NAME = "loadMotor";
    private static final String FLAP_NAME = "flap";

    // In radians
    public static double FLAP_MIN_ERROR = .03;
    public static double FLAP_MIN_ANGLE = 0;
    public static double FLAP_MAX_ANGLE = 0;

    public static double SHOOTER_MIN_ERROR = 0.1;
    public static double SHOOTER_DEFAULT_POWER = .75;

    public static double LOAD_MOTOR_POWER = 1;
    public static double LOAD_MOTOR_DELAY = .5;

    private static final double LAMBDA = 1; // = g / (2*v0*v0) optimal lambda will be found and used

    private DcMotor motor1, motor2, loadMotor;
    private Servo flap;

    private double motor1Power, motor2Power, loadMotorPower;
    private double flapAngle;
    private double targetAngle = -1;

    private Pose2d targetRelPose;

    private Target target = Target.HIGH_GOAL;
    private Alliance alliance = Alliance.BLUE;

    private double shootScheduler = 0;
    private boolean loading = false;

    public Shooter() {
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        TelemetryPacket packet = new TelemetryPacket();

        // Initialize motors and servos //
        motor1 = hardwareMap.get(DcMotor.class, MOTOR1_NAME);
        motor2 = hardwareMap.get(DcMotor.class, MOTOR2_NAME);
        loadMotor = hardwareMap.get(DcMotor.class, LOAD_MOTOR_NAME);

        flap = hardwareMap.get(Servo.class, FLAP_NAME);

        // Reverse direction
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        setShooterPower(0);
        setLoadPower(0);
        setFlapAngle(Math.toRadians(45));
        updateMotorPowers();
        updateServoPositions();

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void update() {
        packet = new TelemetryPacket();

        setShooterPower(SHOOTER_DEFAULT_POWER);

        if (!doneShooting()) {
            updateShooter();
            updateLoadingRing();
        }

        updateMotorPowers();
        updateServoPositions();

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        packet = new TelemetryPacket();

        setShooterPower(0);
        setLoadPower(0);
        setFlapAngle(Math.toRadians(45));
        updateMotorPowers();
        updateServoPositions();

        dashboard.sendTelemetryPacket(packet);
    }

    public boolean readyToShoot() {
        if (targetAngle != -1) {
            return posToAngle(flap.getPosition()) - targetAngle < FLAP_MIN_ERROR &&
                    motor1.getPower() - motor1Power < SHOOTER_MIN_ERROR &&
                    motor2.getPower() - motor2Power < SHOOTER_MIN_ERROR &&
                    !isLoading();
        } else {
            return false;
        }
    }
    public boolean doneShooting() {
        return shootScheduler == 0 && !isLoading();
    }


    public void aimAsync(Pose2d targetRelPose) {
        double dist = Math.hypot(targetRelPose.getY(), targetRelPose.getX());
        double height = target.getLocation(alliance).getZ();

        // Math stuff
        double k = 1.0/(2*dist*LAMBDA);
        double det = 1 - 4*(height - SHOOTER_LOCATION.getZ() + LAMBDA*dist*dist)*LAMBDA;
        double angle = Math.atan(k - k * Math.sqrt(det));

        targetAngle = angle;
        setFlapAngle(targetAngle);
    }


    public void shootAsync(Pose2d targetRelPose) {
        this.targetRelPose = targetRelPose;
        shootScheduler += 1;
    }
    public void updateShooter() {
        // TODO: come up with better solution so that it can get updated pose, may be unnecessary
        updateServoPositions(); //aimAsync(targetRelPose);
        if (readyToShoot() && shootScheduler != 0) {
            startLoadingRing();
            shootScheduler -= 1;
        }
    }
    public void shoot(Pose2d targetRelPose) {
        shootAsync(targetRelPose);
        ElapsedTime elapsedTime = new ElapsedTime();
        while (!doneShooting() || elapsedTime.milliseconds() < TIMEOUT) {
            update();
        }
    }



    ElapsedTime loadingElapse = new ElapsedTime();
    public void startLoadingRing() {
        loading = true;
        loadingElapse.reset();
        setLoadPower(LOAD_MOTOR_POWER);
    }
    public void updateLoadingRing() {
        if (isLoading() && loadingElapse.milliseconds() / 1000.0 > LOAD_MOTOR_DELAY) {
            loading = false;
            setLoadPower(0);
        }
    }
    public boolean isLoading() {
        return loading;
    }



    // Angle is in radians
    public void setFlapAngle(double angle) {
        flapAngle = angle;
    }
    public void setShooterPower(double power) {
        // TODO: Clip powers to make sure it's between -1 and 1
        double p = Range.clip(power, -1, 1);
        motor1Power = motor2Power = p;
    }
    public void setLoadPower(double power) {
        loadMotorPower = Range.clip(power, -1, 1);
    }

    private double posToAngle(double pos) {
        // Scale from 0 = minAngle to 1 = maxAngle
        return Range.scale(pos, 0,1, FLAP_MIN_ANGLE, FLAP_MAX_ANGLE);
    }
    private double angleToPos(double angle) {
        // Scale from minAngle = 0 to maxAngle = 1
        return Range.scale(angle, FLAP_MIN_ANGLE, FLAP_MAX_ANGLE, 0, 1);
    }

    private void updateMotorPowers() {
        motor1.setPower(motor1Power);
        motor2.setPower(motor2Power);
        loadMotor.setPower(loadMotorPower);
        packet.put("Shooter power: ", motor1.getPower());
    }
    private void updateServoPositions() {
        flap.setPosition(angleToPos(flapAngle));
        packet.addLine("Flap Angle: " +  Math.toDegrees(targetAngle) + " target vs " +
                Math.toDegrees(posToAngle(flap.getPosition())) + " actual");
    }

    public void setTarget(Target target) {
        this.target = target;
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }
    public Target getTarget() {
        return target;
    }
    public Alliance setAlliance() {
        return alliance;
    }
}
