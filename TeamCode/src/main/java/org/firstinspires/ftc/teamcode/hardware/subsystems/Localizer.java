package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotlib.hardware.Encoder;
import org.firstinspires.ftc.teamcode.util.Field;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.ARM_DOWN_LOCATION;
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

    private boolean canLaunch = true;

    public static double fudgeFactor = 1;
    public static double launchVel = 8;
    private static final double g = 9.8;

    private static Field.Target target;
    private static Field.Alliance alliance;

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

    /*
     * Frame of refrence: robot center with axis aligned with global axis
     */
    public Vector2d getTargetRelativeLocation2d(Vector2d pos) {
        Vector2d targetPos = target.getLocation2d(alliance);
        return targetPos.minus(pos);
    }
    public Vector2d getTargetRelativeLocation2d() {
        return getTargetRelativeLocation2d(getPoseEstimate().vec());
    }
    public Vector3D getTargetRelativeLocation3D(Vector2d pos) {
        Vector3D targetPos = target.getLocation(alliance);
        Vector3D pos3D = new Vector3D(pos.getX(), pos.getY(), 0);
        return targetPos.subtract(pos3D);
    }
    public Vector3D getTargetRelativeLocation3D() {
        return getTargetRelativeLocation3D(getPoseEstimate().vec());
    }

    public double getTargetHeading(Vector2d vector2d) {
        Vector2d targetRelPos = getTargetRelativeLocation2d(vector2d);
        double theta = targetRelPos.angleBetween(new Vector2d(1, 0));
        double sy = ARM_DOWN_LOCATION.getY();
        double targetMag = targetRelPos.norm();
        double alpha = Math.asin(sy / targetMag);
        return theta + alpha;
    }
    public double getTargetHeading() {
        Vector2d targetRelPos = getTargetRelativeLocation2d();
        double theta = targetRelPos.angleBetween(new Vector2d(1, 0));
        double sy = ARM_DOWN_LOCATION.getY();
        double targetMag = targetRelPos.norm();
        double alpha = Math.asin(sy / targetMag);
        return theta + alpha;
    }
    public double getTargetLaunchAngle(Vector2d vector2d) {
        try {
            final double h = getTargetRelativeLocation3D(vector2d).getZ() - SHOOTER_LOCATION.getZ();
            double d = getTargetRelativeLocation2d().minus(
                    SHOOTER_LOCATION_2d.rotated(
                            getTargetHeading(vector2d)
                    )).norm();

            double k = d * d * g / (2 * launchVel * launchVel);
            return fudgeFactor * Math.atan((d + Math.sqrt(d*d - 4*(h+k)*k))/(2*(h+k))
            );
        } catch (Exception e) {
            canLaunch = false;
            return Math.toRadians(22); // Average launch angle
        }
    }
    public double getTargetLaunchAngle() {
        try {
            final double h = getTargetRelativeLocation3D().getZ() - SHOOTER_LOCATION.getZ();
            double d = getTargetRelativeLocation2d().minus(
                    SHOOTER_LOCATION_2d.rotated(
                            getTargetHeading()
                    )).norm();

            double k = d * d * g / (2 * launchVel * launchVel);
            return fudgeFactor * Math.atan((d + Math.sqrt(d*d - 4*(h+k)*k))/(2*(h+k))
            );
        } catch (Exception e) {
            canLaunch = false;
            return Math.toRadians(22); // Average launch angle
        }
    }

    public boolean canLaunch() {
        return canLaunch;
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
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()),
                encoderTicksToInches(rightEncoder.getRawVelocity()),
                encoderTicksToInches(frontEncoder.getRawVelocity())
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
