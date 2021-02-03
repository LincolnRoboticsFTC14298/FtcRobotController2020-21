package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotlib.hardware.Encoder;
import org.firstinspires.ftc.robotlib.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Field;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotlib.util.MathUtil.poseToVector2D;
import static org.firstinspires.ftc.robotlib.util.MathUtil.rotateVector;
import static org.firstinspires.ftc.robotlib.util.MathUtil.vector3DToVector2D;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.ARM_DOWN_LOCATION;
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

    private boolean canLaunch = true;

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

    public static double fudgeFactor = 1;
    public static double launchVel = 8;
    private static final double g = 9.8;


    public Vector2D getPosition() {
        return poseToVector2D(getPoseEstimate());
    }

    public Vector2D getTargetRelativeLocation2D() {
        // In frame of refrence of robot center with axis aligned with global axis
        Vector2D targetPos = MathUtil.vector3DToVector2D(target.getLocation(alliance));
        Vector2D pos = getPosition();
        return targetPos.subtract(pos);
    }
    public Vector3D getTargetRelativeLocation3D() {
        // In frame of refrence of robot center with axis aligned with global axis
        Vector3D targetPos = target.getLocation(alliance);
        Vector3D pos = MathUtil.vector2DToVector3D(getPosition());
        return targetPos.subtract(pos);
    }

    public double getTargetHeading() {
        double theta = MathUtil.angle(new Vector2D(1, 0), getTargetRelativeLocation2D());
        double sy = ARM_DOWN_LOCATION.getY();
        double targetMag = getTargetRelativeLocation2D().getNorm();
        double alpha = Math.asin(sy / targetMag);
        return theta + alpha;
    }
    public double getTargetLaunchAngle() {
        try {
            final double h = getTargetRelativeLocation3D().getZ() - SHOOTER_LOCATION.getZ();
            double d = getTargetRelativeLocation2D().subtract(
                    rotateVector(
                            vector3DToVector2D(SHOOTER_LOCATION),
                            getPoseEstimate().getHeading()
                    )).getNorm();
            double k = d * d * g / (2 * launchVel * launchVel);
            return fudgeFactor * Math.atan((d + Math.sqrt(d*d - 4*(h+k)*k))/(2*(h+k))
            );
        } catch (Exception e) {
            canLaunch = false;
            return Math.toRadians(22);
        }

    }

    public boolean canLaunch() {
        return canLaunch;
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
