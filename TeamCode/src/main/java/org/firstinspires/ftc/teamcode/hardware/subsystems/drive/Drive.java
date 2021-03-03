package org.firstinspires.ftc.teamcode.hardware.subsystems.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotlib.hardware.roadrunner.MecanumDrive;
import org.firstinspires.ftc.robotlib.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.util.Field;
import org.firstinspires.ftc.teamcode.util.Ring;
import org.firstinspires.ftc.teamcode.util.WobbleGoal;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.ARM_DOWN_LOCATION_2d;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.drive.DriveConstants.encoderTicksToInches;

@Config
public class Drive extends MecanumDrive {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String LEFT_FRONT_NAME = "leftFront";
    private static final String LEFT_REAR_NAME = "leftRear";
    private static final String RIGHT_REAR_NAME = "rightRear";
    private static final String RIGHT_FRONT_NAME = "rightFront";

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static int POSE_HISTORY_LIMIT = 100;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;

    private Localizer localizer;

    public static double LAUNCH_X = Field.LAUNCH_LINE_X - .75 * Field.TILE_WIDTH;
    public static double BEHIND_LINE_ERROR = .5; // inches

    public static double WALL_X = -2.5 * Field.TILE_WIDTH;
    public static double WALL_ERROR = .5; // inches

    public Drive(HardwareMap hardwareMap, Localizer localizer) {
        super("Drive", DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new LinkedList<>();

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_NAME);
        leftRear = hardwareMap.get(DcMotorEx.class, LEFT_REAR_NAME);
        rightRear = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_NAME);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_REAR_NAME);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        this.localizer = localizer;
        setLocalizer(localizer);
    }

    public Drive(HardwareMap hardwareMap) {
        this(hardwareMap, new Localizer(hardwareMap));
    }

    @Override
    public void init() {
        setMotorPowers(0,0,0,0);
    }

    @Override
    public void update() {
        Pose2d currentPose = getPoseEstimate();

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }
                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose, getPoseVelocity()));

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }
    }

    @Override
    public void stop() {
        setMotorPowers(0,0,0,0);
    }

    @Override
    public void updateTelemetry() {
        Pose2d currentPose = getPoseEstimate();

        telemetry.put("mode", mode);

        telemetry.put("Motor velocities: ", getWheelVelocities().toString());

        telemetry.put("x", currentPose.getX());
        telemetry.put("y", currentPose.getY());
        telemetry.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);
                break;
            }
            case FOLLOW_TRAJECTORY: {
                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        Field.ringProvider.draw(fieldOverlay);
        Field.wobbleGoalProvider.draw(fieldOverlay, localizer.getAlliance());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void updateLogging() {
        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        logger.atInfo().log("Mode: %s", mode);

        logger.atInfo().log("Pose: %s", currentPose);

        logger.atInfo().log("Pose Error: %s", lastError);
    }

    /*
     * Input is the forwardAmt, strafeAmt, rotation
     */
    public void teleopControl(Pose2d rawControllerInput, boolean fieldCentric, boolean pointAtTarget) {
        Vector2d input = new Vector2d(rawControllerInput.getX(), rawControllerInput.getY());

        double turn = -rawControllerInput.getHeading();

        if (fieldCentric) {
            input = input.rotated(-getPoseEstimate().getHeading());
        }

        if (pointAtTarget) {
            pointAtTargetAsync();
            turn = 0; // TODO: may not work, must test
        }

        setWeightedDrivePower(
                new Pose2d(input.getX(), input.getY(), turn)
        );
    }

    public void pointAtTargetAsync() {
        turnToAsync(localizer.getTargetHeading());
    }
    public void pointAtTarget() {
        turnTo(localizer.getTargetHeading());
    }

    public void strafeToPointAsync(Pose2d target) {
        Pose2d pose = getPoseEstimate();
        Trajectory trajectory = trajectoryBuilder(pose)
                .lineToSplineHeading(target)
                .build();
        followTrajectoryAsync(trajectory);
    }
    public void strafeToPoint(Pose2d target) {
        Pose2d pose = getPoseEstimate();
        Trajectory trajectory = trajectoryBuilder(pose)
                .lineToSplineHeading(target)
                .build();
        followTrajectory(trajectory);
    }

    public void goBehindLineAsync() {
        // Doesn't need to if already behind line
        Vector2d targetVec;
        if (!isBehindLine()) targetVec = new Vector2d(LAUNCH_X, localizer.getPoseEstimate().getY());
        else targetVec = localizer.getPoseEstimate().vec();

        double angle = localizer.getTargetHeading(targetVec);
        strafeToPointAsync(new Pose2d(targetVec, angle));
    }
    public boolean isBehindLine() {
        return getPoseEstimate().getX() < LAUNCH_X + BEHIND_LINE_ERROR;
    }

    public void goToRingAsync() {
        if (Field.ringProvider.amount() > 0) {
            Vector2d pos = localizer.getPoseEstimate().vec();

            Ring closestRing = Field.ringProvider.getClosest(pos);
            Vector2d ringPos = closestRing.getPosition();
            Vector2d dp = ringPos.minus(pos);
            dp = dp.times(1 - 5.0 / dp.norm()); // 5 inches before
            Vector2d targetPos = pos.plus(dp);
            double heading = dp.angleBetween(new Vector2d(1, 0));

            strafeToPointAsync(new Pose2d(targetPos, heading));
        }
    }
    public void goToRing() {
        goToRingAsync();
        waitForIdle();
    }

    public void goToWobbleGoalAsync() {
        if (Field.wobbleGoalProvider.amount() > 0) {
            Vector2d pos = localizer.getPoseEstimate().vec();

            WobbleGoal closestWobbleGoal = Field.wobbleGoalProvider.getClosest(pos);
            Vector2d wobbleGoalPos = closestWobbleGoal.getPosition();
            Vector2d dp = wobbleGoalPos.minus(pos);
            Vector2d armPos = ARM_DOWN_LOCATION_2d;
            dp = dp.times(1 - armPos.norm() / dp.norm());
            Vector2d targetPos = pos.plus(dp);
            double heading = armPos.angleBetween(new Vector2d(1, 0)) + dp.angleBetween(new Vector2d(1, 0));

            strafeToPointAsync(new Pose2d(targetPos, heading));
        }
    }
    public void goToWobbleGoal() {
        goToRingAsync();
        waitForIdle();
    }
    public boolean isAtWobbleGoal() {
        return Field.wobbleGoalProvider.atWobbleGoal(getPoseEstimate());
    }

    public void goToWallAsync() {
        Vector2d targetVec = new Vector2d(WALL_X, localizer.getPoseEstimate().getY());
        strafeToPointAsync(new Pose2d(targetVec, Math.PI));
    }
    public void goToWall() {
        goToWallAsync();
        waitForIdle();
    }
    public boolean isAtWall() {
        return getPoseEstimate().getX() < WALL_X + WALL_ERROR;
    }



    // RoadRunner Methods //
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );

        turnStart = clock.seconds();
        mode = Mode.TURN;
    }
    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void turnToAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );

        turnStart = clock.seconds();
        mode = Mode.TURN;
    }
    public void turnTo(double angle) {
        turnToAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }
    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void cancelFollowing() {
        mode = Mode.IDLE;
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }
}