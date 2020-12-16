package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.PositionProvider;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.util.Field.Alliance;
import org.firstinspires.ftc.teamcode.util.Field.Target;

import robotlib.hardware.RobotBase;


public class Robot extends RobotBase {
    public PositionProvider positionProvider;

    // Subsystems
    public Vision vision;
    public Intake intake;
    public Shooter shooter;
    public Arm arm;
    public Drive drive;

    public Target target = Target.HIGH_GOAL;
    public Alliance alliance = Alliance.BLUE;

    public Robot(OpMode opMode) {
        super(opMode);

        positionProvider = new PositionProvider();

        vision = new Vision(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, positionProvider);
        arm = new Arm(hardwareMap);
        drive = new Drive(hardwareMap, positionProvider);

        subsystemManager.add(vision);
        subsystemManager.add(intake);
        subsystemManager.add(shooter);
        subsystemManager.add(arm);
        subsystemManager.add(drive);
    }

    @Override
    public void start() {
        subsystemManager.start();
        telemetry.setMsTransmissionInterval(50);
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
        drive.setMotorPowers(0,0,0,0);

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
        //turret.aimAtTargetAsync();
        if (shootScheduler > 0) {
            drive.pointAtTargetAsync();
            shooter.aimAsync(); // Start aiming before aligned, doesn't need to be fully aligned
            shooter.turnOnShooterMotor();
            //elevator.raiseAsync();
            if (positionProvider.readyToShoot() && drive.readyToShoot() && shooter.readyToLaunch() ) {
                // && turret.isAligned() && elevator.isUp(), remove drive.readyToShoot()
                shooter.launchAsync();
                launching = true;
            }
            if (launching && shooter.isRetracted()) {
                launching = false;
                shootScheduler--;
            }
        } else {
            shooter.turnOffShooterMotor();
        }
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
