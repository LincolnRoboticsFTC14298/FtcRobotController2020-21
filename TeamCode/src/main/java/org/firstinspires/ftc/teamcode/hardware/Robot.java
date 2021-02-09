package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotlib.hardware.RobotBase;
import org.firstinspires.ftc.robotlib.util.MathUtil;
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
import org.firstinspires.ftc.teamcode.util.Ring;
import org.firstinspires.ftc.teamcode.util.WobbleGoal;

import static org.firstinspires.ftc.robotlib.util.MathUtil.poseToVector2D;
import static org.firstinspires.ftc.robotlib.util.MathUtil.vector2DToPose;
import static org.firstinspires.ftc.robotlib.util.MathUtil.vector3DToVector2D;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.ARM_DOWN_LOCATION;


public class Robot extends RobotBase {
    public Localizer localizer;

    // Subsystems
    public Vision vision;
    public Arm arm;
    public Intake intake;
    public RingCounter ringCounter;
    public Shooter shooter;
    public Drive drive;

    public Target target = Target.HIGH_GOAL;

    // TODO: Maybe alliance in field class?
    public Alliance alliance = Alliance.BLUE;

    public enum ControlMode {
        MANUAL,
        COLLECTING,
        TRAVELING_TO_SHOOT
    }

    ControlMode controlMode = ControlMode.MANUAL;


    public Robot(OpMode opMode) {
        super(opMode);

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
        Field.ringProvider.update(localizer.getPoseEstimate());

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



    // Navigation //
    public void goToRing() {
        if (Field.ringProvider.getRings().size() > 0) {
            Ring closestRing = Field.ringProvider.getClosest(localizer.getPoseEstimate());
            Vector2D pos = poseToVector2D(localizer.getPoseEstimate());
            Vector2D ringPos = closestRing.getPosition();

            Vector2D dp = ringPos.subtract(pos);
            dp = dp.scalarMultiply(1 - 5.0 / dp.getNorm()); // 5 inches before
            Vector2D targetPos = pos.add(dp);
            double heading = MathUtil.angle(new Vector2D(1, 0), dp);

            drive.strafeToPointAsync(vector2DToPose(targetPos, heading));
        }
    }
    public void goToWobbleGoal() {
        if (Field.wobbleGoalProvider.getWobbleGoals().size() > 0) {
            WobbleGoal closestWobbleGoal = Field.wobbleGoalProvider.getClosest(localizer.getPoseEstimate());
            Vector2D pos = poseToVector2D(localizer.getPoseEstimate());
            Vector2D ringPos = closestWobbleGoal.getPosition();

            Vector2D dp = ringPos.subtract(pos);
            Vector2D armPos = vector3DToVector2D(ARM_DOWN_LOCATION);
            dp = dp.scalarMultiply(1 - armPos.getNorm() / dp.getNorm());
            Vector2D targetPos = pos.add(dp);
            double heading = MathUtil.angle(new Vector2D(1, 0), armPos) + MathUtil.angle(new Vector2D(1, 0), dp);

            drive.strafeToPointAsync(vector2DToPose(targetPos, heading));
        }
    }

    public void updateNavigation() {
        vision.scan();
        switch (controlMode) {
            case MANUAL:
                break;
            case COLLECTING:
                if (ringCounter.getNumberOfRings() == 3) {
                    drive.cancelFollowing();
                    controlMode = ControlMode.TRAVELING_TO_SHOOT;
                } else if (!drive.isBusy()) {
                    if (Field.ringProvider.getRings().size() > 0) {
                        goToRing();
                    } else {
                        // TODO: rotate to find rings
                    }
                }
                break;
            case TRAVELING_TO_SHOOT:
                if (drive.isBehindLine()) {
                    // Can make async
                    // TODO: Check time and shoot target dependent on time
                    shoot(3);
                    controlMode = ControlMode.COLLECTING.COLLECTING;
                } else if (!drive.isBusy()) {
                    drive.goBehindLineAsync();
                }
                break;
        }
    }


    // Getters and setters //
    public void setManualMode() {
        controlMode = ControlMode.MANUAL;
        if (!drive.isBusy()) {
            drive.cancelFollowing();
        }
    }
    public void setAutoMode() {
        controlMode = ControlMode.COLLECTING;
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
