package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import robotlib.hardware.Subsystem;

@Config
public class Turret implements Subsystem {
    private static final double GEAR_RATIO = .1; // turret rot / motor rot

    private static final int INITIAL_LOCAL_HEADING = 0; // 0 local heading is aligned with heading of drive

    private static final int TICKS_PER_REV = 0;

    private static final double SPEED = 1;

    public static PIDFCoefficients POS_PIDF = new PIDFCoefficients(0,0,0,0);

    private static final String TURRET_MOTOR_NAME = "turret";
    private static final double TURRET_MIN_ERROR = .1;

    private PositionProvider positionProvider;

    private DcMotorEx turretMotor;

    private double targetGlobalHeading;

    enum Mode {
        IDLE,
        AIMING,
        AUTO_AIMING
    }

    private Mode mode;

    public Turret(HardwareMap hardwareMap, PositionProvider positionProvider) {
        turretMotor = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);

        // Needs to move clockwise

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, POS_PIDF);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        this.positionProvider = positionProvider;
    }

    @Override
    public void start() {
        stopAiming();
    }

    @Override
    public void update() {
        switch (mode) {
            case AUTO_AIMING:
                aimAtTargetAsync();
                break;
            default:
                break;
        }
    }

    @Override
    public void stop() {
        aim(INITIAL_LOCAL_HEADING);
        stopAiming();
    }

    @Override
    public void updateMotorsAndServos() {
        switch (mode) {
            case AUTO_AIMING:
            case AIMING:
                turretMotor.setPower(SPEED);
                break;
            default:
                turretMotor.setPower(0);
                break;
        }
    }

    public boolean isAligned() {
        return Math.abs(getTurretGlobalHeading() - targetGlobalHeading) < TURRET_MIN_ERROR;
    }

    public void aimAtTargetAsync() {
        // TODO: adjust for horizontal movement relative to target;
        aimAsync(positionProvider.getTargetHeading());
        mode = Mode.AUTO_AIMING;
    }

    public void aimAsync(double globalHeading) {
        mode = Mode.AIMING;
        targetGlobalHeading = globalHeading;
        setTurretGlobalTargetHeading(globalHeading);
    }
    public void aim(double globalHeading) {
        aimAsync(globalHeading);
        while (!isAligned()) {
            updateMotorsAndServos();
        }
        stopAiming();
    }
    public void stopAiming() {
        mode = Mode.IDLE;
    }

    private void setTurretGlobalTargetHeading(double heading) {
        setTurretLocalTargetHeading(heading - getTurretGlobalHeading());
    }
    private double getTurretGlobalHeading() {
        return getTurretLocalHeading() + positionProvider.getPoseEstimate().getHeading();
    }


    // When local = 0, local heading is aligned with robot,
    private void setTurretLocalTargetHeading(double heading) {
        double targetTurretGearRev = (heading - INITIAL_LOCAL_HEADING) / (2 * Math.PI);
        double targetMotorGearRev = targetTurretGearRev / GEAR_RATIO;
        int targetTicks = (int) (targetMotorGearRev * TICKS_PER_REV);
        turretMotor.setTargetPosition(targetTicks);
    }
    private double getTurretLocalHeading() {
        double motorRevs = getMotorRevs();
        double turretRevs = motorRevs * GEAR_RATIO;
        return 2 * Math.PI * turretRevs + INITIAL_LOCAL_HEADING;
    }


    private double getMotorRevs() {
        int ticks = turretMotor.getCurrentPosition();
        return (double) ticks / TICKS_PER_REV;
    }
}
