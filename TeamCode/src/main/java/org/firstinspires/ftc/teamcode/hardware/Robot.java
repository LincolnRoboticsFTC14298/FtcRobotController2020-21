package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.hardware.RobotBase;
import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RingCounter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.util.Field.Alliance;
import org.firstinspires.ftc.teamcode.util.Field.Target;

import java.util.LinkedList;
import java.util.Queue;


public class Robot implements RobotBase {
    private Telemetry telemetry;

    private SubsystemManager subsystemManager = new SubsystemManager();

    // Subsystems //
    public Localizer localizer;

    public Vision vision;
    public Arm arm;
    public Intake intake;
    public RingCounter ringCounter;
    public Shooter shooter;
    public Drive drive;

    public Target target = Target.HIGH_GOAL;

    // TODO: Maybe alliance in field class?
    public Alliance alliance = Alliance.BLUE;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        localizer = new Localizer(hardwareMap);
        ringCounter = new RingCounter(hardwareMap);

        vision = new Vision(hardwareMap, localizer);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap, ringCounter);
        shooter = new Shooter(hardwareMap, localizer);
        drive = new Drive(hardwareMap, localizer);

        subsystemManager.add(vision);
        subsystemManager.add(arm);
        subsystemManager.add(intake);
        subsystemManager.add(ringCounter);
        subsystemManager.add(shooter);
        subsystemManager.add(drive);
    }

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        subsystemManager.init();
        updateTelemetry();
    }

    @Override
    public void initUpdate() {
        subsystemManager.initUpdate();
        updateTelemetry();
    }

    @Override
    public void start() {
        subsystemManager.start();
        updateTelemetry();
    }

    @Override
    public void update() {
        localizer.update();

        updateShooting();

        subsystemManager.update();
        updateTelemetry();
    }

    @Override
    public void stop() {
        subsystemManager.stop();
        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addData("Alliance: ", alliance.toString());
        telemetry.addData("Pose:     ", localizer.getPoseEstimate().toString());
        telemetry.addData("Target:   ", target.toString());
        telemetry.update();
    }



    // Shooting //
    private Queue<Target> shootingQueue = new LinkedList<>();
    public boolean doneShooting() {
        return shootingQueue.isEmpty();
    }
    public void waitUntilDoneShooting() {
        while (!doneShooting() && !Thread.currentThread().isInterrupted()) {
            update();
        }
    }

    public void shootAsync(Target target) {
        shootingQueue.add(target);
    }
    public void shootAsync(Target target, int n) {
        for (int i = 0; i < n; i++)
            shootAsync(target);
    }
    public void shootAsync() {
        shootAsync(Target.HIGH_GOAL);
    }
    public void shootAsync(int n) {
        shootAsync(Target.HIGH_GOAL, n);
    }

    public void powerShot() {
        // TODO: test strafing shot vs rotate shot
        // Outward shot
        shootAsync(Target.OUTWARD_POWER_SHOT);

        // Middle shot
        shootAsync(Target.MIDDLE_POWER_SHOT);

        // Inward shot
        shootAsync(Target.INWARD_POWER_SHOT);
    }
    public void cancelShot() {
        shootingQueue.clear();
    }


    public enum ShootingStatus {
        IDLE,
        AIMING,
        SHOOTING
    }

    ShootingStatus shootingStatus = ShootingStatus.IDLE;
    public void updateShooting() {
        shooter.aimAsync();

        if (doneShooting()) {
            shootingStatus = ShootingStatus.IDLE;
            drive.cancelFollowing();
            shooter.turnOffShooterMotor();
        }
        switch (shootingStatus) {
            case IDLE:
                if (!doneShooting()) {
                    shootingStatus = ShootingStatus.AIMING;
                    drive.stop();

                    setTarget(shootingQueue.peek());
                    drive.pointAtTargetAsync();
                    shooter.turnOnShooterMotor();
                }
                break;
            case AIMING:
                if (localizer.canLaunch() && shooter.readyToLaunch() && !drive.isBusy()) {
                    shootingStatus = ShootingStatus.SHOOTING;
                    shooter.launchAsync();
                }
                break;
            case SHOOTING:
                if (shooter.isRetractedStatus()) {
                    ringCounter.removeRingFromCartridge();
                    shootingQueue.remove();
                    shootingStatus = ShootingStatus.IDLE;
                    setTarget(Target.HIGH_GOAL); // default target
                }
                break;
        }
    }


    // Getters and setters //
    public void setPoseEstimate(Pose2d pose) {
        localizer.setPoseEstimate(pose);
    }

    public Target getTarget() {
        return target;
    }
    public void setTarget(Target target) {
        this.target = target;
        localizer.setTarget(target);
    }

    public Alliance getAlliance() {
        return alliance;
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        localizer.setAlliance(alliance);
    }
}
