package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.robotlib.hardware.Encoder;
import org.firstinspires.ftc.teamcode.util.Field;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.robotlib.util.MathUtil.inchesToMeters;
import static org.firstinspires.ftc.teamcode.robotlib.util.MathUtil.poseToVector3D;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.SHOOTER_LOCATION;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class Localizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 0;
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder frontEncoder;

    public Localizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    private static Field.Target target;
    private static Field.Alliance alliance;

    private static double targetHeading = 0;
    private static double targetLaunchAngle = 0;

    public static double fudgeFactor = 1;
    public static double launchVel = 8;
    private static final double g = 9.8;
    private static final Vector3D G = new Vector3D(0, 0, -g/2);

    private static boolean recentlyUpdated = true;

    public void update() {
        findTargetAngles();
    }

    public Vector3D getShooterLocation() {
        return poseToVector3D(getPoseEstimate()).add(SHOOTER_LOCATION);
    }
    public Vector3D getTargetRelativeLocation() {
        // (0,0,0) is shooter position
        Vector3D targetPos = target.getLocation(alliance);
        Vector3D shooterPos = getShooterLocation();
        return targetPos.subtract(shooterPos);
    }

    public double getTargetHeading() {
        return targetHeading; // Happens to be in the heading frame
    }
    public double getTargetLaunchAngle() {
        return targetLaunchAngle; // Happens to be in the heading frame
    }



    private final static double relativeAccuracy = 1.0e-8;
    private final static double absoluteAccuracy = 1.0e-6;
    private final static LaguerreSolver solver = new LaguerreSolver(relativeAccuracy, absoluteAccuracy);
    private Vector3D estimatedRingPosition(double t) {
        Vector3D robotVelocity = inchesToMeters(poseToVector3D(getPoseVelocity()));
        Vector3D velocityVector = robotVelocity.add(targetLaunchVector(t));
        return G.scalarMultiply(t*t)
                .add(
                        velocityVector.scalarMultiply(t)
                );
    }
    private Vector3D targetLaunchVector(double t) {
        Vector3D robotVelocity = inchesToMeters(poseToVector3D(getPoseVelocity()));
        Vector3D targetVector = inchesToMeters(getTargetRelativeLocation());
        // T/t - VelR - G*t
        return targetVector.scalarMultiply(1/t)
                .subtract(robotVelocity)
                .subtract(G.scalarMultiply(t));
    }
    private double findTimeHitTarget() {
        Vector3D target = inchesToMeters(getTargetRelativeLocation());
        Vector3D robotVelocity = inchesToMeters(poseToVector3D(getPoseVelocity()));

        double[] coefficients = new double[5];
        coefficients[0] = target.dotProduct(target);
        coefficients[1] = -2.0 * (target.dotProduct(robotVelocity));
        coefficients[2] = robotVelocity.dotProduct(robotVelocity) + g*target.getZ() - launchVel*launchVel;
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

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()),
                encoderTicksToInches(rightEncoder.getRawVelocity()),
                encoderTicksToInches(frontEncoder.getRawVelocity())
        );
    }

    public Field.Target getTarget() {
        return target;
    }
    public void setTarget(Field.Target target) {
        Localizer.target = target;
    }

    public Field.Alliance getAlliance() {
        return alliance;
    }
    public void setAlliance(Field.Alliance alliance) {
        Localizer.alliance = alliance;
    }
}
