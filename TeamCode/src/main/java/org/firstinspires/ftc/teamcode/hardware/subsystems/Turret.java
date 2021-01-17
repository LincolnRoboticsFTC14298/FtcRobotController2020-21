package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.robotlib.hardware.Subsystem;
import org.firstinspires.ftc.teamcode.robotlib.util.MathUtil;

@Config
public class Turret extends Subsystem {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final double GEAR_RATIO = .1; // turret output revs / motor input revs

    private static final int INITIAL_LOCAL_HEADING = 0; // 0 local heading is aligned with heading of drive

    private static final int TICKS_PER_REV = 0;

    private static final double SPEED = 1;

    public static PIDFCoefficients POS_PIDF = new PIDFCoefficients(0,0,0,0);

    private static final String TURRET_MOTOR_NAME = "turret";
    private static final double TURRET_MIN_ERROR = .1;

    private Localizer localizer;

    private DcMotorEx turretMotor;

    private double targetGlobalHeading;

    enum Mode {
        IDLE,
        AIMING,
        AUTO_AIMING
    }

    private Mode mode;

    public Turret(HardwareMap hardwareMap, Localizer localizer) {
        super("Turret");

        turretMotor = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);

        // Needs to move clockwise

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, POS_PIDF);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        this.localizer = localizer;
    }

    @Override
    public void init() {
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
    public void updateMotorAndServoValues() {
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

    @Override
    public void updateTelemetry() {
        telemetry.put("mode: ", mode);
    }

    @Override
    public void updateLogging() {
        logger.atFine().log("mode: ", mode);
        logger.atFinest().log("is aligned: ", isAligned());
    }

    public boolean isAligned() {
        return Math.abs(getTurretGlobalHeading() - targetGlobalHeading) < TURRET_MIN_ERROR;
    }

    public void aimAtTargetAsync() {
        aimAsync(localizer.getTargetHeading());
        mode = Mode.AUTO_AIMING;
    }

    public void aimAsync(double globalHeading) {
        mode = Mode.AIMING;
        targetGlobalHeading = globalHeading;
        // could be setTurretLocalTargetHeading(positionProvider.getRelativeTargetHeading());
        setTurretGlobalTargetHeading(globalHeading);
    }
    public void aim(double globalHeading) {
        aimAsync(globalHeading);
        while (!isAligned() && !Thread.currentThread().isInterrupted()) {
            updateMotorAndServoValues();
        }
        stopAiming();
    }
    public void stopAiming() {
        mode = Mode.IDLE;
    }

    private void setTurretGlobalTargetHeading(double heading) {
        turn(heading - getTurretGlobalHeading());
    }
    private double getTurretGlobalHeading() {
        return getTurretLocalHeading() + localizer.getPoseEstimate().getHeading();
    }

    // Local heading x axis is aligned with robot,
    private void setTurretLocalTargetHeading(double heading) {
       turn(heading - getTurretLocalHeading());
    }
    private double getTurretLocalHeading() {
        double motorRevs = getMotorRevs();
        double turretRevs = motorRevs * GEAR_RATIO;
        return 2 * Math.PI * turretRevs + INITIAL_LOCAL_HEADING;
    }

    private void turn(double deltaHeading) {
        deltaHeading = MathUtil.angleWrapRadians(deltaHeading);
        double deltaTargetTurretGearRev = deltaHeading / (2 * Math.PI);
        double deltaTargetMotorGearRev = deltaTargetTurretGearRev / GEAR_RATIO;
        int targetTicks = turretMotor.getCurrentPosition() + (int) (deltaTargetMotorGearRev * TICKS_PER_REV);
        turretMotor.setTargetPosition(targetTicks);
    }

    private double getMotorRevs() {
        int ticks = turretMotor.getCurrentPosition();
        return (double) ticks / TICKS_PER_REV;
    }
}