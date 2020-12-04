package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.util.Field;
import robotlib.util.MathUtil;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.SHOOTER_LOCATION;

public class PositionProvider {
    private Pose2d poseEstimate;
    private Field.Target target;
    private Field.Alliance alliance;

    public void setPoseEstimate(Pose2d poseEstimate) {
        this.poseEstimate = poseEstimate;
    }
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public Vector3D getShooterLocation() {
        return new Vector3D(
                poseEstimate.getX() + SHOOTER_LOCATION.getX(),
                poseEstimate.getY() + SHOOTER_LOCATION.getY(),
                   SHOOTER_LOCATION.getZ()
        );
    }

    public Vector3D getTargetRelativeLocation() {
        // Rel pose of target in reference to the shooter
        Vector3D targetPos = target.getLocation(alliance);
        Vector3D shooterPos = getShooterLocation();
        return new Vector3D(targetPos.getX() - shooterPos.getX(),
                targetPos.getY() - shooterPos.getY(),
                targetPos.getZ() - shooterPos.getZ());
    }

    public double getTargetHeading() {
        Vector3D diff = getTargetRelativeLocation();
        return Math.atan2(diff.getY(), diff.getX()); // Happens to be in the heading frame
    }
    public double getTargetRelativeHeading() {
        // In reference to the the heading of the robot
        return MathUtil.angleWrapRadians(getTargetHeading() - poseEstimate.getHeading());
    }

    public Field.Target getTarget() {
        return target;
    }
    public void setTarget(Field.Target target) {
        this.target = target;
    }

    public Field.Alliance getAlliance() {
        return alliance;
    }
    public void setAlliance(Field.Alliance alliance) {
        this.alliance = alliance;
    }
}
