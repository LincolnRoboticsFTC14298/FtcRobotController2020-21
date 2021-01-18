package org.firstinspires.ftc.teamcode.robotlib.hardware.roadrunner;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;

import static java.util.Collections.emptyList;

public abstract class MecanumDrive extends Drive {
    protected double kV, kA, kStatic, trackWidth, wheelBase, lateralMultiplier;

    public MecanumDrive(String name, 
                        double kV,
                        double kA,
                        double kStatic,
                        double trackWidth) {
        super(name);
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.trackWidth = trackWidth;
        this.wheelBase = trackWidth;
        this.lateralMultiplier = 1.0;
    }
    public MecanumDrive(String name,
                        double kV,
                        double kA,
                        double kStatic,
                        double trackWidth,
                        double wheelBase,
                        double lateralMultiplier) {
        super(name);
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
        this.lateralMultiplier = lateralMultiplier;
    }

    /**
     * Default localizer for mecanum drives based on the drive encoders and (optionally) a heading sensor.
     *
     * drive1 drive
     * useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     */
    class MecanumLocalizer implements Localizer {
        private MecanumDrive drive1;
        private boolean useExternalHeading;

        public MecanumLocalizer(MecanumDrive drive1) {
            this.drive1 = drive1;
            this.useExternalHeading = true;
        }
        public MecanumLocalizer(MecanumDrive drive1, boolean useExternalHeading) {
            this.drive1 = drive1;
            this.useExternalHeading = useExternalHeading;
        }

        private Pose2d _poseEstimate = new Pose2d();

        @Override
        public Pose2d getPoseEstimate() {
            return _poseEstimate;
        }

        @Override
        public void setPoseEstimate(@NotNull Pose2d pose2d) {
            lastWheelPositions = emptyList();
            lastExtHeading = Double.NaN;
            if (useExternalHeading) drive1.externalHeading = pose2d.getHeading();
            _poseEstimate = pose2d;
        }

        private Pose2d poseVelocity = null;

        @Nullable
        @Override
        public Pose2d getPoseVelocity() {
            return poseVelocity;
        }
        public void setPoseVelocity(Pose2d value) {
            poseVelocity = value;
        }
        private List<Double> lastWheelPositions = emptyList();
        private double lastExtHeading = Double.NaN;
        
        @Override
        public void update() {
            List<Double> wheelPositions = drive1.getWheelPositions();
            double extHeading = Double.NaN;
            if (useExternalHeading) {
                extHeading = drive1.getExternalHeading();
            }
            if (!lastWheelPositions.isEmpty()) {
                List<Double> wheelDeltas = new ArrayList<>();
                for (int i = 0; i < wheelPositions.size(); i++) {
                    wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
                }
                Pose2d robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                        wheelDeltas, drive1.trackWidth, drive1.wheelBase, drive1.lateralMultiplier
                );
                
                double finalHeadingDelta;
                if (useExternalHeading) finalHeadingDelta = Angle.normDelta(extHeading - lastExtHeading);
                else finalHeadingDelta = robotPoseDelta.getHeading();
                
                _poseEstimate = Kinematics.relativeOdometryUpdate(
                    _poseEstimate,
                    new Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
                );
            }

            List<Double> wheelVelocities = drive1.getWheelVelocities();
            double extHeadingVel = drive1.getExternalHeadingVelocity();
            if (wheelVelocities != null) {
                poseVelocity = MecanumKinematics.wheelToRobotVelocities(
                        wheelVelocities, drive1.trackWidth, drive1.wheelBase, drive1.lateralMultiplier
                );
                if (useExternalHeading && !Double.isNaN(extHeadingVel)) {
                    poseVelocity = new Pose2d(poseVelocity.vec(), extHeadingVel);
                }
            }

            lastWheelPositions = wheelPositions;
            lastExtHeading = extHeading;
        }
    }


    Localizer localizer = new MecanumLocalizer(this);

    public void setDriveSignal(DriveSignal driveSignal) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocities(
                driveSignal.getVel(), trackWidth, wheelBase, lateralMultiplier);
        List<Double> accelerations = MecanumKinematics.robotToWheelAccelerations(
                driveSignal.getAccel(), trackWidth, wheelBase, lateralMultiplier);
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    public void setDrivePower(Pose2d drivePower) {
        List<Double> powers = MecanumKinematics.robotToWheelVelocities(
                drivePower, 1.0, 1.0, lateralMultiplier);
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    public abstract void setMotorPowers(double frontLeft, double rearLeft, double rearRight, double frontRight);

    /**
     * Returns the positions of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    protected abstract List<Double> getWheelPositions();

    /**
     * Returns the velocities of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    public List<Double> getWheelVelocities() {
        return null;
    }
}
