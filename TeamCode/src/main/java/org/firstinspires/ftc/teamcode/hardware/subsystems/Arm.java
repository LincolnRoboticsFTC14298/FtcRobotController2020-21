package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import robotlib.hardware.Subsystem;

@Config
public class Arm extends Subsystem {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    //private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    public static final String CLAW_SERVO_NAME = "claw";
    public static final String ARM_SERVO_NAME = "arm";

    // Position from 0 to 1
    public static double CLAW_OPEN_POSITION = 1;
    public static double CLAW_CLOSE_POSITION = 0;

    public static double ARM_LIFT_POSITION = 1;
    public static double ARM_DEFAULT_POSITION = .5;
    public static double ARM_LOWER_POSITION = 0;

    private Servo armServo, clawServo;
    private double armPosition, clawPosition;

    public Arm(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, CLAW_SERVO_NAME);
        armServo = hardwareMap.get(Servo.class, ARM_SERVO_NAME);
    }

    @Override
    public void start() {
        closeClaw(); // At the beginning of the round, the claw is closed with the wobble
        defaultArm();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        closeClaw();
        defaultArm();
    }

    public void openClaw() {
        setClawPosition(CLAW_OPEN_POSITION);
    }
    public void closeClaw() {
        setClawPosition(CLAW_CLOSE_POSITION);
    }

    public void liftArm() {
        setArmPosition(ARM_LIFT_POSITION);
    }
    public void defaultArm() {
        setArmPosition(ARM_DEFAULT_POSITION);
    }
    public void lowerArm() {
        setArmPosition(ARM_LOWER_POSITION);
    }

    public void setArmPosition(double position) {
        this.armPosition = position;
    }
    public void setClawPosition(double position) {
        this.clawPosition = position;
    }

    @Override
    public void updateMotorsAndServos() {
        armServo.setPosition(armPosition);
        clawServo.setPosition(clawPosition);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm position: ", armServo.getPosition());
        packet.put("Claw position: ", clawServo.getPosition());
        dashboard.sendTelemetryPacket(packet);
    }
}
