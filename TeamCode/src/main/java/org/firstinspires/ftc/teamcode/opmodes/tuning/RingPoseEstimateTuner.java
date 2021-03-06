package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.optim.univariate.UnivariateOptimizer;
import org.apache.commons.math3.optim.univariate.UnivariatePointValuePair;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.robotlib.util.MathUtil;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.vision.RingData;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotlib.util.MathUtil.squareError;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.CAMERA_LOCATION;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.CAMERA_PITCH;

@Config
@TeleOp(name="Ring Pose Estimate Tuner", group="Tuner")
@Disabled
public class RingPoseEstimateTuner extends OpMode {
    public static double fudgeFactorX = 1, fudgeFactorY = 1;

    public static Vector2d ringVector = new Vector2d(0,0);
    public static Pose2d robotInitPose = new Pose2d(0,0,0);

    private final ArrayList<Vector2d> predictedVector2ds = new ArrayList<>();
    private final ArrayList<Vector2d> actualVector2ds = new ArrayList<>();

    public Telemetry telemetry;

    private Robot robot;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad = new RadicalGamepad(gamepad1);

        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.setPoseEstimate(robotInitPose);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        gamepad.update();

        robot.vision.analyze();

        Pose2d input = new Pose2d(-gamepad.left_stick_y, -gamepad.left_stick_x, gamepad.right_stick_x);
        robot.drive.teleopControl(input, true, true);

        RingData ring = robot.vision.getRingData().get(0);
        Vector3D predictedVector3D = robot.vision.getRingCameraLocalPosition(ring);
        Vector2d predictedVector = MathUtil.vector3DToVector2d(predictedVector3D);

        // robot pose -> camera position
        Pose2d pose = robot.localizer.getPoseEstimate();
        Vector2d local = MathUtil.globalToLocal(ringVector, pose);
        Vector3D local3D = MathUtil.vector2dToVector3D(local); // Frame of camera, axis pitch is global
        Vector3D cameraLocal = MathUtil.rotateY(local3D.subtract(CAMERA_LOCATION), -CAMERA_PITCH); // Frame of camera, axis pitch local
        Vector2d actualVector = MathUtil.vector3DToVector2d(cameraLocal);

        if (gamepad.a) {
            predictedVector2ds.add(predictedVector);
            actualVector2ds.add(actualVector);
        } else if (gamepad.b) {
            fudgeFactorX = findLambdaX();
            fudgeFactorY = findLambdaY();
        }

        robot.update();

        telemetry.addData("Predicted Vector: ", predictedVector3D.toString());
        telemetry.addData("Actual Vector: ", cameraLocal.toString());
        telemetry.addData("Fudge Factor X: ", fudgeFactorX);
        telemetry.addData("Fudge Factor Y: ", fudgeFactorY);
        telemetry.update();
    }

    UnivariateOptimizer optimizer = new BrentOptimizer(1e-10, 1e-14);

    UnivariateFunction fX = new ErrorFunctionX();
    public double findLambdaX() {
        UnivariatePointValuePair val = optimizer.optimize(new MaxEval(200),
                new UnivariateObjectiveFunction(fX),
                GoalType.MINIMIZE,
                new SearchInterval(0.01, 2));

        return val.getValue();
    }

    UnivariateFunction fY = new ErrorFunctionY();
    public double findLambdaY() {
        UnivariatePointValuePair val = optimizer.optimize(new MaxEval(200),
                new UnivariateObjectiveFunction(fY),
                GoalType.MINIMIZE,
                new SearchInterval(0.01, 2));

        return val.getValue();
    }

    private class ErrorFunctionX implements UnivariateFunction {
        @Override
        public double value(double x) {
            double totalError = 0.0;
            for (int i = 0; i < predictedVector2ds.size(); i++) {
                double predicted = x * predictedVector2ds.get(i).getX();
                totalError += squareError(predicted, actualVector2ds.get(i).getX());
            }
            return totalError;
        }
    }

    private class ErrorFunctionY implements UnivariateFunction {
        @Override
        public double value(double y) {
            double totalError = 0.0;
            for (int i = 0; i < predictedVector2ds.size(); i++) {
                double predicted = y * predictedVector2ds.get(i).getY();
                totalError += squareError(predicted, actualVector2ds.get(i).getY());
            }
            return totalError;
        }
    }
}

