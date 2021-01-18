package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotlib.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RingCounter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Turret;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.util.Field;
import org.firstinspires.ftc.teamcode.util.Field.Alliance;
import org.firstinspires.ftc.teamcode.util.Field.Target;
import org.firstinspires.ftc.teamcode.util.Ring;
import org.firstinspires.ftc.teamcode.util.WobbleGoal;

import static org.firstinspires.ftc.robotlib.util.MathUtil.poseToVector2D;
import static org.firstinspires.ftc.robotlib.util.MathUtil.vector2DToPose;


public class Robot extends RobotBase {
    public Localizer localizer;

    // Subsystems
    public Vision vision;
    public Arm arm;
    public Intake intake;
    public Elevator elevator;
    public RingCounter ringCounter;
    public Turret turret;
    public Shooter shooter;
    public Drive drive;

    public Target target = Target.HIGH_GOAL;

    // TODO: Maybe alliance in field class?
    public Alliance alliance = Alliance.BLUE;

    public enum Mode {
        MANUAL,
        COLLECTING,
        TRAVELING_TO_SHOOT
    }

    Mode mode = Mode.MANUAL;

    public Robot(OpMode opMode) {
        super(opMode);

        localizer = new Localizer(hardwareMap);

        vision = new Vision(hardwareMap, localizer);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        elevator = new Elevator(hardwareMap);
        ringCounter = new RingCounter(hardwareMap);
        turret = new Turret(hardwareMap, localizer);
        shooter = new Shooter(hardwareMap, localizer);
        drive = new Drive(hardwareMap, localizer);

        subsystemManager.add(vision);
        subsystemManager.add(arm);
        subsystemManager.add(intake);
        subsystemManager.add(elevator);
        subsystemManager.add(ringCounter);
        subsystemManager.add(turret);
        subsystemManager.add(shooter);
        subsystemManager.add(drive);
    }

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        subsystemManager.init();
        telemetry.update();
    }

    @Override
    public void initUpdate() {
        subsystemManager.initUpdate();
        telemetry.update();
    }

    @Override
    public void start() {
        subsystemManager.start();
        telemetry.update();
    }

    @Override
    public void update() {
        localizer.update();
        Field.updateRings(localizer.getPoseEstimate());

        switch (mode) {
            case MANUAL:
                break;
            case COLLECTING:
                if (ringCounter.getNumberOfRings() == 3) {
                    drive.cancelFollowing();
                    mode = Mode.TRAVELING_TO_SHOOT;
                } else if (!drive.isBusy()) {
                    vision.scan();
                    if (Field.rings.size() > 0) {
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
                    mode = Mode.COLLECTING;
                } else if (!drive.isBusy()) {
                    drive.goBehindLineAsync();
                }
                break;
        }

        updateShooting();

        subsystemManager.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        subsystemManager.stop();
        telemetry.update();
    }


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
    public void shoot(int n) {
        shootAsync(n);
        waitUntilDoneShooting();
    }

    boolean launching = false;
    public void updateShooting() {
        // Maybe only want to aim when shootScheduler > 0
        turret.aimAtTargetAsync();
        shooter.aimAsync();

        if (launching && shooter.isRetractedStatus()) {
            launching = false;
            shootScheduler--;
        }

        if (shootScheduler > 0) {
            //drive.pointAtTargetAsync();
            shooter.turnOnShooterMotor();
            elevator.raiseAsync();
            if (localizer.readyToShoot() && !launching && shooter.readyToLaunch()
                    && turret.isAligned() && elevator.isUp()) {
                shooter.launchAsync();
                launching = true;
            }
        } else {
            shooter.turnOffShooterMotor();
            elevator.lowerAsync();
        }
    }

    public void goToRing() {
        if (Field.rings.size() > 0) {
            Ring closestRing = Field.getClosestRing(localizer.getPoseEstimate());
            Pose2d pose = localizer.getPoseEstimate();
            Vector2D pos = poseToVector2D(pose);
            double heading = Vector2D.angle(pos, closestRing.getPosition()) - pose.getHeading();
            Vector2D ringPos = closestRing.getPosition();
            Vector2D targetPos = ringPos.add(ringPos.normalize().scalarMultiply(-5)); // 5 inches before
            drive.strafeToPointAsync(vector2DToPose(targetPos, heading));
        }
    }

    public void goToWobbleGoal() {
        WobbleGoal closestWobbleGoal = Field.getClosestWobbleGoal(localizer.getPoseEstimate());
        Pose2d pose = localizer.getPoseEstimate();
        Vector2D pos = poseToVector2D(pose);
        double heading = Vector2D.angle(pos, closestWobbleGoal.getPosition()) - pose.getHeading();
        Vector2D wobbleGoalPos = closestWobbleGoal.getPosition();
        Vector2D targetPos = wobbleGoalPos.add(wobbleGoalPos.normalize().scalarMultiply(-5)); // 5 inches before
        drive.strafeToPointAsync(vector2DToPose(targetPos, heading));
    }

    public void setManualMode() {
        mode = Mode.MANUAL;
        if (!drive.isBusy()) {
            drive.cancelFollowing();
        }
    }
    public void setAutoMode() {
        mode = Mode.COLLECTING;
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
