package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.compenents.Arm;
import org.firstinspires.ftc.teamcode.hardware.compenents.Drive;
import org.firstinspires.ftc.teamcode.hardware.compenents.Intake;
import org.firstinspires.ftc.teamcode.hardware.compenents.PositionLocalizer;
import org.firstinspires.ftc.teamcode.hardware.compenents.RoadRunner;
import org.firstinspires.ftc.teamcode.hardware.compenents.Shooter;
import org.firstinspires.ftc.teamcode.hardware.compenents.Vision;
import org.firstinspires.ftc.teamcode.hardware.util.RobotBase;
import org.firstinspires.ftc.teamcode.util.Field.*;


public class Robot extends RobotBase {
    // Subsystems
    public Drive drive = new Drive(this);
    public Intake intake = new Intake(this);
    public Shooter shooter = new Shooter(this);
    public Arm arm = new Arm(this);
    public PositionLocalizer positionLocalizer = new PositionLocalizer(this);
    public RoadRunner roadRunner = new RoadRunner(this);
    public Vision vision = new Vision(this);

    public boolean autoAim = false, localControl = true, liftArm = true;
    public Target target = Target.HIGH_GOAL;
    public Alliance alliance = Alliance.BLUE;

    @Override
    public void init(OpMode opMode) {
        super.init(opMode);

        subsystemManager.add(drive);
        subsystemManager.add(intake);
        subsystemManager.add(shooter);
        subsystemManager.add(arm);
        subsystemManager.add(positionLocalizer);
        subsystemManager.add(roadRunner);
        subsystemManager.add(vision);

        subsystemManager.init();
        telemetry.update();
    }

    @Override
    public void teleopUpdate() {
        // LOGIC HERE PLEASE


        // Drive


        // Intake



        // Shooter

    }

    @Override
    public void update() {
        subsystemManager.update();
        telemetry.update();
    }

    @Override
    public void end() {
        subsystemManager.end();
        telemetry.update();
    }

    public void setTarget(Target target) {
        drive.setTarget(target);
        shooter.setTarget(target);
    }
}
