package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.hardware.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.RingData;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotlib.util.MathUtil.squareError;

@Config
@TeleOp(name="Ring Pose Estimate Tuner No Drive", group="Tuner")
@Disabled
public class RingPoseEstimateTunerNoDrive extends OpMode {
    public static double fudgeFactorX = 1, fudgeFactorY = 1;
    public static Vector2d currentVector2d = new Vector2d(0,0);

    private final ArrayList<Vector2d> predictedVector2ds = new ArrayList<>();
    private final ArrayList<Vector2d> actualVector2ds = new ArrayList<>();

    public Telemetry telemetry;

    private Vision vision;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);
        gamepad = new RadicalGamepad(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {
        gamepad.update();

        vision.analyze();

        RingData ring = vision.getRingData().get(0);
        Vector3D predVector3D = vision.getRingCameraLocalPosition(ring);
        Vector2d predVector = MathUtil.vector3DToVector2d(predVector3D);

        if (gamepad.a) {
            predictedVector2ds.add(predVector);
            actualVector2ds.add(currentVector2d);
        } else if (gamepad.b) {
            fudgeFactorX = findLambdaX();
            fudgeFactorY = findLambdaY();
        }

        vision.update();

        telemetry.addData("Predicted Vector: ", predVector3D.toString());
        telemetry.addData("Actual Vector: ", currentVector2d.toString());
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
