package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotlib.hardware.Encoder;
import org.firstinspires.ftc.robotlib.hardware.roadrunner.ThreeTrackingWheelLocalizer;
import org.firstinspires.ftc.robotlib.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Field;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.ARM_DOWN_LOCATION;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.ARM_MIDDLE_LOCATION_2d;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.SHOOTER_LOCATION;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.SHOOTER_LOCATION_2d;

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
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String LEFT_ENCODER_NAME = "leftEncoder",
            RIGHT_ENCODER_NAME = "rightEncoder",
            FRONT_ENCODER_NAME = "frontEncoder";

    public static final double TICKS_PER_REV = 0;
    public static final double WHEEL_RADIUS = 2; // in
    public static final double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    public static double fudgeFactor = 1;
    public static double launchVel = 30; // ft / s
    private static final double g = 32.1741; // ft / s^2

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder frontEncoder;

    private boolean canLaunch = true;

    private static Field.Target target;
    private static Field.Alliance alliance;

    private double targetHeading;
    private double targetLaunchAngle;

    public Localizer(HardwareMap hardwareMap) {
        super("Localizer", Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LEFT_ENCODER_NAME));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RIGHT_ENCODER_NAME));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, FRONT_ENCODER_NAME));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    @Override
    public void update() {
        super.update();
        targetHeading = getTargetHeading(getPoseEstimate().vec());
        targetLaunchAngle = getTargetLaunchAngle(getPoseEstimate().vec());
    }

    @Override
    public void stop() {

    }

    @Override
    public void updateTelemetry() {
        telemetry.put("Can launch", canLaunch());
    }

    @Override
    public void updateLogging() {
        logger.atInfo().log("Target heading: %.3f", targetHeading);
        logger.atInfo().log("Target angle: %.3f", Math.toDegrees(targetLaunchAngle));
    }

    /*
     * Frame of reference: robot center with global axis
     */
    public Vector3D getRelativeTargetLocation3D(Vector2d pos) {
        Vector3D targetPos = target.getLocation(alliance);
        Vector3D pos3D = MathUtil.vector2dToVector3D(pos);
        return targetPos.subtract(pos3D);
    }
    public Vector3D getRelativeTargetLocation3D() {
        return getRelativeTargetLocation3D(getPoseEstimate().vec());
    }
    public Vector2d getRelativeTargetLocation2d(Vector2d pos) {
        return MathUtil.vector3DToVector2d(getRelativeTargetLocation3D(pos));
    }
    public Vector2d getRelativeTargetLocation2d() {
        return getRelativeTargetLocation2d(getPoseEstimate().vec());
    }

    public double getTargetHeading(Vector2d vector2d) {
        Vector2d targetRelPos = getRelativeTargetLocation2d(vector2d);
        double theta = targetRelPos.angleBetween(new Vector2d(1, 0));
        double sy = ARM_DOWN_LOCATION.getY();
        double targetMag = targetRelPos.norm();
        double alpha = Math.asin(sy / targetMag);
        return theta + alpha;
    }
    public double getTargetHeading() {
        return targetHeading;
    }

    /*
     * Frame of reference: shooter center with global axis
     */
    public Vector3D getRelativeShooterTargetLocation3D(Vector2d pos) {
        Vector3D targetPos = target.getLocation(alliance);
        Vector2d shooter2d = MathUtil.localToGlobal(SHOOTER_LOCATION_2d, new Pose2d(pos, getTargetHeading(pos)));
        Vector3D shooter3D = MathUtil.vector2dToVector3D(shooter2d, SHOOTER_LOCATION.getZ());
        return targetPos.subtract(shooter3D);
    }
    public Vector3D getRelativeShooterTargetLocation3D() {
        return getRelativeShooterTargetLocation3D(getPoseEstimate().vec());
    }

    public double getTargetLaunchAngle(Vector2d vector2d) {
        try {
            Vector3D relTarget = getRelativeShooterTargetLocation3D(vector2d);
            double h = relTarget.getZ();
            double d = MathUtil.vector3DToVector2d(relTarget).norm();

            double k = d * d * g / (2 * launchVel * launchVel);
            return fudgeFactor * Math.atan((d + Math.sqrt(d*d - 4*(h+k)*k))/(2*(h+k))
            );
        } catch (Exception e) {
            canLaunch = false;
            return Math.toRadians(22); // Average launch angle
        }
    }
    public double getTargetLaunchAngle() {
        return targetLaunchAngle;
    }

    public boolean canLaunch() {
        return canLaunch;
    }

    public Vector2d getWobbleGoalDropOffPos() {
        Pose2d pose = getPoseEstimate();
        Vector2d local = ARM_MIDDLE_LOCATION_2d.rotated(pose.getHeading());
        return local.plus(pose.vec());
    }

    // RoadRunner Methods //
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
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }

    // Getters and Setters //
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
