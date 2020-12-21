package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PositionProvider;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Turret;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.util.Field.Alliance;
import org.firstinspires.ftc.teamcode.util.Field.Target;

import robotlib.hardware.RobotBase;


public class Robot extends RobotBase {
    public PositionProvider positionProvider;

    // Subsystems
    public Vision vision;
    public Arm arm;
    public Intake intake;
    public Elevator elevator;
    public Turret turret;
    public Shooter shooter;
    public Drive drive;

    public Target target = Target.HIGH_GOAL;
    public Alliance alliance = Alliance.BLUE;

    public Robot(OpMode opMode) {
        super(opMode);

        positionProvider = new PositionProvider();

        vision = new Vision(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        elevator = new Elevator(hardwareMap);
        turret = new Turret(hardwareMap, positionProvider);
        shooter = new Shooter(hardwareMap, positionProvider);
        drive = new Drive(hardwareMap, positionProvider);

        subsystemManager.add(vision);
        subsystemManager.add(arm);
        subsystemManager.add(intake);
        subsystemManager.add(elevator);
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
        positionProvider.update();

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
            update();
        }
    }

    public void shoot(int n) {
        shootScheduler += n;
    }
    public void shootTarget(Target target, int n) {
        setTarget(target);
        shoot(n);
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
            if (positionProvider.readyToShoot() && !launching && shooter.readyToLaunch()
                    && turret.isAligned() && elevator.isUp()) {
                shooter.launchAsync();
                launching = true;
            }
        } else {
            shooter.turnOffShooterMotor();
            elevator.lowerAsync();
        }
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
        positionProvider.setPoseEstimate(pose);
    }

    public Target getTarget() {
        return target;
    }
    public void setTarget(Target target) {
        this.target = target;
        positionProvider.setTarget(target);
    }

    public Alliance getAlliance() {
        return alliance;
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        positionProvider.setAlliance(alliance);
    }
}
