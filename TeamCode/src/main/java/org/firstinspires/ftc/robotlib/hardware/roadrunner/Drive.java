package org.firstinspires.ftc.robotlib.hardware.roadrunner;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;

public abstract class Drive extends AbstractSubsystem {
    public Drive(String name) {
        super(name);
    }

    /**
     * Localizer used to determine the evolution of [poseEstimate].
     */
    protected Localizer localizer;
    public Localizer getLocalizer() {
        return localizer;
    }
    public void setLocalizer(Localizer localizer) {
        this.localizer = localizer;
    }

    private double headingOffset = 0.0;

    /**
     * The raw heading used for computing [externalHeading]. Not affected by [externalHeading] setter.
     */
    protected double rawExternalHeading;
    public abstract double getRawExternalHeading();

    /**
     * The robot's heading in radians as measured by an external sensor (e.g., IMU, gyroscope).
     */
    double externalHeading;
    public double getExternalHeading() {
        return Angle.norm(getRawExternalHeading() + headingOffset);
    }
    public void setExternalHeading(double value) {
        headingOffset = -getRawExternalHeading() + value;
    }

    /**
     * The robot's current pose estimate.
     */
    Pose2d poseEstimate;
    public Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }
    public void setPoseEstimate(Pose2d value) {
        localizer.setPoseEstimate(value);
    }

    /**
     *  Current robot pose velocity (optional)
     */
    Pose2d poseVelocity;
    public Pose2d getPoseVelocity() {
        return localizer.getPoseVelocity();
    }

    /**
     * Updates [poseEstimate] with the most recent positional change.
     */
    public void updatePoseEstimate() {
        localizer.update();
    }

    /**
     * Sets the current commanded drive state of the robot. Feedforward is applied to [driveSignal] before it reaches
     * the motors.
     */
    protected abstract void setDriveSignal(DriveSignal driveSignal);

    /**
     * Sets the current commanded drive state of the robot. Feedforward is *not* applied to [drivePower].
     */
    protected abstract void setDrivePower(Pose2d drivePower);

    /**
     * The heading velocity used to determine pose velocity in some cases
     */
    public double getExternalHeadingVelocity() {
        return Double.NaN;
    }
}
