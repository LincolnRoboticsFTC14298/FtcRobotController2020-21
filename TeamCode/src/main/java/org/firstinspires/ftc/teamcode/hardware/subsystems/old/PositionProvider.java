package org.firstinspires.ftc.teamcode.hardware.subsystems.old;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.util.Field;

import org.firstinspires.ftc.teamcode.robotlib.util.MathUtil;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.SHOOTER_LOCATION;

@Config
@Deprecated
public class PositionProvider {
    private static Pose2d poseEstimate;
    private static Pose2d velEstimate;
    private static Field.Target target;
    private static Field.Alliance alliance;

    private static double targetHeading = 0;
    private static double targetLaunchAngle = 0;

    public static double fudgeFactor = 1;
    public static double launchVel = 8;
    private static final double g = 9.8;
    private static final Vector3D G = new Vector3D(0, 0, -g/2);

    private static boolean recentlyUpdated = true;

    public void setPoseEstimate(Pose2d poseEstimate) {
        this.poseEstimate = poseEstimate;
    }
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public void setVelEstimate(Pose2d velEstimate) {
        this.velEstimate = velEstimate;
    }
    public Pose2d getVelEstimate() {
        return velEstimate;
    }

    public void update() {
        findTargetAngles();
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
        // TargetPos - shooterPos
        Vector3D targetPos = target.getLocation(alliance);
        Vector3D shooterPos = getShooterLocation();
        return new Vector3D(targetPos.getX() - shooterPos.getX(),
                targetPos.getY() - shooterPos.getY(),
                targetPos.getZ() - shooterPos.getZ());
    }
    public Pose2d getTargetRelativePose2d() {
        Vector3D targetRelLoc = getTargetRelativeLocation();
        double heading = getTargetRelativeHeading();
        return new Pose2d(targetRelLoc.getX(), targetRelLoc.getY(), heading);
    }

    public double getTargetDistance() {
        Vector3D diff = getTargetRelativeLocation();
        return Math.hypot(diff.getX(), diff.getY());
    }
    public double getTargetHeading() {
        return targetHeading; // Happens to be in the heading frame
    }
    public double getTargetRelativeHeading() {
        // In reference to the the heading of the robot
        return MathUtil.angleWrapRadians(getTargetHeading() - poseEstimate.getHeading());
    }
    public double getTargetLaunchAngle() {
        return targetLaunchAngle; // Happens to be in the heading frame
    }



    private final static double relativeAccuracy = 1.0e-8;
    private final static double absoluteAccuracy = 1.0e-6;
    private final static LaguerreSolver solver = new LaguerreSolver(relativeAccuracy, absoluteAccuracy);
    private Vector3D estimatedRingPosition(double t) {
        Vector3D velR = new Vector3D(velEstimate.getX(),velEstimate.getY(),0);
        return G.scalarMultiply(t*t)
                .add(
                        velR.add(
                                targetLaunchVector(t)
                        ).scalarMultiply(t));
    }
    private Vector3D targetLaunchVector(double t) {
        Vector3D robotVelocity = new Vector3D(velEstimate.getX(),velEstimate.getY(),0);
        Vector3D targetVector = getTargetRelativeLocation();
        // (T/t - VelR - G*t)/v
        return targetVector.scalarMultiply(1/t)
                .subtract(robotVelocity)
                .subtract(G.scalarMultiply(t));
    }
    private double findTimeHitTarget() {
        Vector3D target = getTargetRelativeLocation();
        Vector3D robotVel = new Vector3D(velEstimate.getX(), velEstimate.getY(),0);

        double[] coefficients = new double[5];
        coefficients[0] = target.dotProduct(target);
        coefficients[1] = -2.0 * (target.dotProduct(robotVel));
        coefficients[2] = robotVel.dotProduct(robotVel) + g*target.getZ() - launchVel*launchVel;
        coefficients[3] = 0.0;
        coefficients[4] = g*g / 4.0;
        PolynomialFunction function = new PolynomialFunction(coefficients);

        return solver.solve(15, function, 0, 1, .05);
    }
    public void findTargetAngles() {
        try {
            double t = findTimeHitTarget();
            Vector3D vel = targetLaunchVector(t);
            targetHeading = Math.atan2(vel.getY(), vel.getX());
            targetLaunchAngle = fudgeFactor*Math.asin(vel.getZ() / launchVel);
            recentlyUpdated = true;
        } catch (Exception e) {
            recentlyUpdated = false;
        }
    }

    public boolean readyToShoot() {
        return recentlyUpdated;
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
