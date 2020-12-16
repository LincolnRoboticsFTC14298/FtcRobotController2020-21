package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import robotlib.hardware.Subsystem;

@Config
public class Elevator implements Subsystem {
    private static final int RADIUS = 5; // inches

    private static final int TICKS_PER_REV = 0;

    private static final double SPEED = 1;

    public static PIDFCoefficients POS_PIDF = new PIDFCoefficients(0,0,0,0);

    private static final String ELEVATOR_MOTOR_NAME = "elevator";
    private static final double HEIGHT_MIN_ERROR = .1;

    DcMotorEx elevatorMotor;


    private double topHeight = 10; // inches

    enum Mode {
        IDLE,
        LOWERING,
        RAISING
    }

    Mode mode = Mode.IDLE;

    public Elevator(HardwareMap hardwareMap, PositionProvider positionProvider) {
        elevatorMotor = hardwareMap.get(DcMotorEx.class, ELEVATOR_MOTOR_NAME);

        // Needs to move clockwise

        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, POS_PIDF);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void start() {
        lower();
    }

    @Override
    public void update() {
        switch (mode) {
            case RAISING:
                if (isUp()) {
                    stopLifting();
                }
                break;
            case LOWERING:
                if (isDown()) {
                    stopLifting();
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void stop() {
        lower();
    }

    @Override
    public void updateMotorsAndServos() {
        switch (mode) {
            case RAISING:
            case LOWERING:
                elevatorMotor.setPower(SPEED);
                break;
            default:
                elevatorMotor.setPower(0);
        }
    }

    public boolean isUp() {
        return Math.abs(getPlatformHeight() - topHeight) < HEIGHT_MIN_ERROR;
    }
    public boolean isDown() {
        return Math.abs(getPlatformHeight() - 0) < HEIGHT_MIN_ERROR;
    }

    public void raiseAsync() {
        mode = Mode.RAISING;
        setPlatformHeight(topHeight);
    }
    public void raise() {
        raise();
        while (!isUp() && !Thread.currentThread().isInterrupted()) {
            update();
            updateMotorsAndServos();
        }
    }

    public void lowerAsync() {
        mode = Mode.LOWERING;
        setPlatformHeight(0);
    }
    public void lower() {
        lowerAsync();
        while (!isDown() && !Thread.currentThread().isInterrupted()) {
            update();
            updateMotorsAndServos();
        }
    }

    public void stopLifting() {
        mode = Mode.IDLE;
    }

    private void setPlatformHeight(double height) {
        double targetRevs = height / (2 * Math.PI * RADIUS);
        int targetTicks = (int) (targetRevs * TICKS_PER_REV);
        elevatorMotor.setTargetPosition(targetTicks);
    }
    private double getPlatformHeight() {
        double revs = getMotorRevs();
        return 2 * Math.PI * RADIUS * revs;
    }

    private double getMotorRevs() {
        int ticks = elevatorMotor.getCurrentPosition();
        return (double) ticks / TICKS_PER_REV;
    }
}
