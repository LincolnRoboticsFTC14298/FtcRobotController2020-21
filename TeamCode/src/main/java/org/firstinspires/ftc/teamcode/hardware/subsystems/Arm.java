package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;

@Config
public class Arm implements Subsystem {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet;

    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    public static final String CLAW_SERVO_NAME = "claw";
    public static final String ARM_SERVO_NAME = "arm";

    // Position from 0 to 1
    public static double CLAW_OPEN_POSITION = 1;
    public static double CLAW_CLOSE_POSITION = 0;

    public static double ARM_LIFT_POSITION = 1;
    public static double ARM_DEFAULT_POSITION = .5;
    public static double ARM_LOWER_POSITION = 0;

    private Servo armServo , clawServo;
    private double armPosition, clawPosition;

    public Arm() {

    }

    @Override
    public void init(HardwareMap hardwareMap) {
        packet = new TelemetryPacket();

        clawServo = hardwareMap.get(Servo.class, CLAW_SERVO_NAME);
        armServo = hardwareMap.get(Servo.class, ARM_SERVO_NAME);

        closeClaw(); // At the beginning of the round, the claw is closed with the wobble
        defaultArm();
        updateServoPositions();

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void update() {
        packet = new TelemetryPacket();

        updateServoPositions();

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        packet = new TelemetryPacket();

        closeClaw();
        defaultArm();
        updateServoPositions();

        dashboard.sendTelemetryPacket(packet);
    }



    public void openClaw() {
        setClawServo(CLAW_OPEN_POSITION);
    }
    public void closeClaw() {
        setClawServo(CLAW_CLOSE_POSITION);
    }

    public void liftArm() {
        setArmAngle(ARM_LIFT_POSITION);
    }
    public void defaultArm() {
        setArmAngle(ARM_DEFAULT_POSITION);
    }
    public void lowerArm() {
        setArmAngle(ARM_LOWER_POSITION);
    }

    public void setArmAngle(double position) {
        this.armPosition = position;
    }

    public void setClawServo(double position) {
        this.clawPosition = position;
    }

    public void updateServoPositions() {
        armServo.setPosition(armPosition);
        clawServo.setPosition(clawPosition);

        packet.put("Arm position: ", armServo.getPosition());
        packet.put("Claw position: ", clawServo.getPosition());
    }
}
