package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Drive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Field.Alliance;
import org.firstinspires.ftc.teamcode.util.Field.Target;

import robotlib.hardware.RobotBase;


public class Robot extends RobotBase {
    // Subsystems
    public Vision vision;
    public Intake intake;
    public Shooter shooter;
    public Arm arm;
    public Drive drive;

//    public static interface PositionProvider {
//        int getX();
//        int getY();
//    }
//
//    private class ThisPositionProvider {
//
//    }

//    public Drive drive = new Drive();
//    public RoadRunnerDrive roadRunnerDrive = new RoadRunnerDrive(drive);



    public boolean autoAim = false, localControl = true, liftArm = true;
    public Target target = Target.HIGH_GOAL;
    public Alliance alliance = Alliance.BLUE;

    public Robot(OpMode opMode) {
        super(opMode);

        vision = new Vision(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        arm = new Arm(hardwareMap);
        drive = new Drive(hardwareMap);

        subsystemManager.add(vision);
        subsystemManager.add(intake);
        subsystemManager.add(shooter);
        subsystemManager.add(arm);
        subsystemManager.add(drive);

        init();
    }

    @Override
    public void init() {
        subsystemManager.init();
        telemetry.update();
    }

    @Override
    public void update() {
        subsystemManager.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        subsystemManager.stop();
        telemetry.update();
    }

    public void shoot(int n) {
        // Not async as to prevent other movements.
        drive.pointAtTargetAsync();
        shooter.aimAsync(drive.getTargetPose()); // Start aiming before aligned, doesn't need to be fully aligned
        ElapsedTime elapsedTime = new ElapsedTime();
        while(drive.isBusy() && elapsedTime.milliseconds() < RobotMap.TIMEOUT) {
            drive.update();
            shooter.update();
            // TODO: if bumped into by robot while shooting, it will not update
        }
        for (int i = 0; i < n; i++) {
            shooter.shoot(drive.getTargetPose());
        }
    }
    public void shootTarget(Target target, int n) {
        setTarget(target);
        shoot(n);
    }
    public void powerShot() {
        // Point to outward power shot
        shootTarget(Target.OUTWARD_POWER_SHOT, 1);
        shootTarget(Target.MIDDLE_POWER_SHOT, 1);
        shootTarget(Target.INWARD_POWER_SHOT, 1);
    }


    public Target getTarget() {
        return target;
    }
    public void setTarget(Target target) {
        this.target = target;
        drive.setTarget(target);
        shooter.setTarget(target);
    }

    public Alliance getAlliance() {
        return alliance;
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        drive.setAlliance(alliance);
        shooter.setAlliance(alliance);
    }
}
