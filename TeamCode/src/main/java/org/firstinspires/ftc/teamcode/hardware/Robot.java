package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.hardware.util.RobotBase;
import org.firstinspires.ftc.teamcode.util.Field.Alliance;
import org.firstinspires.ftc.teamcode.util.Field.Target;


public class Robot extends RobotBase {
    // Subsystems
    public Vision vision = new Vision();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public Arm arm = new Arm();

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

    public NewDrive drive = new NewDrive();

    public boolean autoAim = false, localControl = true, liftArm = true;
    public Target target = Target.HIGH_GOAL;
    public Alliance alliance = Alliance.BLUE;

    public Robot() {
        subsystemManager.add(vision);
        subsystemManager.add(intake);
        subsystemManager.add(shooter);
        subsystemManager.add(arm);
        subsystemManager.add(drive);
    }

    @Override
    public void init(OpMode opMode) {
        super.init(opMode);
        subsystemManager.init(hardwareMap);
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
        shooter.aimAsync(drive.getTargetRelPose()); // Start aiming before aligned, doesn't need to be fully aligned
        ElapsedTime elapsedTime = new ElapsedTime();
        while(drive.isBusy() && elapsedTime.milliseconds() < RobotMap.TIMEOUT) {
            drive.update();
            shooter.update();
            // TODO: if bumped into by robot while shooting, it will not update
        }
        for (int i = 0; i < n; i++) {
            shooter.shoot(drive.getTargetRelPose());
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


    public void setTarget(Target target) {
        this.target = target;
        drive.setTarget(target);
        shooter.setTarget(target);
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        drive.setAlliance(alliance);
        shooter.setAlliance(alliance);
    }
    public Target getTarget() {
        return target;
    }
    public Alliance getAlliance() {
        return alliance;
    }
}
