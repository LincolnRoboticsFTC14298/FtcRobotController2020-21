package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotlib.hardware.RobotBase;
import org.firstinspires.ftc.robotlib.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RingCounter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.util.Field;
import org.firstinspires.ftc.teamcode.util.Field.Alliance;
import org.firstinspires.ftc.teamcode.util.Field.Target;


public class Robot extends RobotBase {
    public Localizer localizer;

    // Subsystems //
    public Vision vision;
    public Arm arm;
    public Intake intake;
    public RingCounter ringCounter;
    public Shooter shooter;
    public Drive drive;

    public Target target = Target.HIGH_GOAL;

    // TODO: Maybe alliance in field class?
    public Alliance alliance = Alliance.BLUE;

    enum ControlMode {
        MANUAL,
        AUTOMATIC
    }

    ControlMode controlMode = ControlMode.MANUAL;

    public Robot(OpMode opMode) {
        super(opMode);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        localizer = new Localizer(hardwareMap);

        vision = new Vision(hardwareMap, localizer);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        ringCounter = new RingCounter(hardwareMap);
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

        updateNavigation();
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
        telemetry.addData("Mode:     ", controlMode);
        telemetry.addData("Target:   ", target.toString());
        telemetry.update();
    }



    // Navigation //
    public enum NavigationStatus {
        COLLECTING,
        TRAVELING_TO_SHOOT,
        SHOOTING,
        TRAVELING_TO_WOBBLE_GOAL,
        TRAVELING_TO_DROP_OFF
    }

    NavigationStatus navigationStatus = NavigationStatus.COLLECTING;
    public void updateNavigation() {
        vision.scan();
        switch (navigationStatus) {
            case COLLECTING:
                if (timeSinceStart.seconds() > 90 && Field.wobbleGoalProvider.amount() > 0) {
                    // Last 30 seconds of match
                    arm.lower();
                    drive.cancelFollowing();
                    navigationStatus = NavigationStatus.TRAVELING_TO_WOBBLE_GOAL;
                } else if (ringCounter.getNumberOfRings() == 3) {
                    drive.cancelFollowing();
                    navigationStatus = NavigationStatus.TRAVELING_TO_SHOOT;
                }
                switch (controlMode) {
                    case AUTOMATIC:
                        Field.ringProvider.update(localizer.getPoseEstimate()); // may have to be in main update loop
                        if (!drive.isBusy() && Field.ringProvider.getRings().size() > 0) {
                           drive.goToRingAsync();
                        } else if (!drive.isBusy()) {
                            // TODO: rotate to find rings
                        }
                        break;
                    case MANUAL:
                        break;
                }
                break;
            case TRAVELING_TO_SHOOT:
                if (!drive.isBusy() && drive.isBehindLine()) {
                    navigationStatus = NavigationStatus.SHOOTING;
                    // TODO: REWRITE???
                    if (timeSinceStart.seconds() > 90) {
                        shootAsync(3);
                    } else {
                        powerShot();
                    }
                } else if (!drive.isBusy()) {
                    drive.goBehindLineAsync();
                }
                break;
            case SHOOTING:
                if (doneShooting()) {
                    navigationStatus = NavigationStatus.COLLECTING;
                }
                break;
            case TRAVELING_TO_WOBBLE_GOAL:
                if (!drive.isBusy() && drive.isAtWobbleGoal()) {
                    Field.wobbleGoalProvider.update(localizer.getPoseEstimate());
                    arm.closeClaw();
                    // May need to sleep or some PICKING_UP state
                    arm.lift();
                    navigationStatus = NavigationStatus.TRAVELING_TO_DROP_OFF;
                } else if (!drive.isBusy()) {
                    drive.goToWobbleGoalAsync();
                }
                break;
            case TRAVELING_TO_DROP_OFF:
                if (!drive.isBusy() && drive.isAtWall()) {
                    arm.openClaw();
                    // May need to sleep or some IS_DROPPING state
                    arm.lower();
                    if (Field.wobbleGoalProvider.amount() > 0) {
                        navigationStatus = NavigationStatus.TRAVELING_TO_WOBBLE_GOAL;
                    } else {
                        arm.closeClaw();
                        navigationStatus = NavigationStatus.COLLECTING;
                    }
                } else if (!drive.isBusy()) {
                    drive.goToWallAsync();
                }
                break;
        }
    }



    // Shooting //
    private int shootScheduler = 0;
    public boolean doneShooting() {
        return shootScheduler == 0;
    }
    public void waitUntilDoneShooting() {
        while (!doneShooting() && !Thread.currentThread().isInterrupted()) {
            // TODO: Find way to use update
            localizer.update();
            updateShooting();
            subsystemManager.update();
            telemetry.update();
        }
    }

    public void shootAsync(int n) {
        shootScheduler += n;
    }
    public void shoot(int n) {
        shootAsync(n);
        waitUntilDoneShooting();
    }
    public void shootTarget(Target target, int n) {
        setTarget(target);
        shootAsync(n);
    }
    public void powerShot() {
        // TODO: Make async
        // TODO: test strafing shot vs rotate shot
        // Break
        drive.stop();

        // Outward shot
        shootTarget(Target.OUTWARD_POWER_SHOT, 1);
        waitUntilDoneShooting();

        // Middle shot
        shootTarget(Target.MIDDLE_POWER_SHOT, 1);
        waitUntilDoneShooting();

        // Inward shot
        shootTarget(Target.INWARD_POWER_SHOT, 1);
        waitUntilDoneShooting();
    }
    public void cancelShot() {
        shootScheduler = 0;
    }


    public enum ShootingStatus {
        IDLE,
        AIMING,
        SHOOTING
    }

    ShootingStatus shootingStatus = ShootingStatus.IDLE;
    public void updateShooting() {
        shooter.aimAsync();

        if (shootScheduler == 0) {
            shootingStatus = ShootingStatus.IDLE;
            drive.cancelFollowing();
            shooter.turnOffShooterMotor();
        }
        switch (shootingStatus) {
            case IDLE:
                if (shootScheduler > 0) {
                    shootingStatus = ShootingStatus.AIMING;
                    drive.stop();
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
                    shootScheduler--;
                    shootingStatus = ShootingStatus.AIMING;
                    drive.pointAtTargetAsync();
                    shooter.turnOnShooterMotor();
                }
                break;
        }
    }


    // Getters and setters //
    public void setManualMode() {
        switch (navigationStatus) {
            case COLLECTING:
                drive.cancelFollowing();
                break;
        }
        controlMode = ControlMode.MANUAL;
    }
    public void setAutoMode() {
        controlMode = ControlMode.AUTOMATIC;
    }

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
