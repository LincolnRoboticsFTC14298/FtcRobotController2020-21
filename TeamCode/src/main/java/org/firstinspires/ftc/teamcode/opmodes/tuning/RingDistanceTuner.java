package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.optim.univariate.UnivariateOptimizer;
import org.apache.commons.math3.optim.univariate.UnivariatePointValuePair;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.vision.RingData;

import java.util.ArrayList;

import robotlib.hardware.gamepad.RadicalGamepad;

import static robotlib.util.MathUtil.squareError;

@Config
@TeleOp(name="Ring Distance Tuner", group="Tuner")
public class RingDistanceTuner extends OpMode {
    public static double fudgeFactor = 1;
    public static double dist = 1; // inches

    private ArrayList<Double> predictedDistances = new ArrayList<>();
    private ArrayList<Double> actualDistances = new ArrayList<>();

    private Robot robot;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        robot = new Robot(this);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        RingData ring = robot.vision.getRings().get(0);
        double predDist = robot.vision.getRingDistance(ring);

        if (gamepad.a) {
            predictedDistances.add(predDist);
            actualDistances.add(dist);
        } else if (gamepad.b) {
            fudgeFactor = findLambda();
        }

        robot.update();

        telemetry.addData("Actual Distance: ", dist);
        telemetry.addData("Predicted Distance: ", fudgeFactor*predDist);
        telemetry.addData("Ring angle", robot.vision.getRingAngle(ring));
        telemetry.addData("Fudge Factor: ", fudgeFactor);
        telemetry.update();
    }

    public double findLambda() {
        UnivariateFunction f = new ErrorFunction();
        UnivariateOptimizer optimizer = new BrentOptimizer(1e-10, 1e-14);

        UnivariatePointValuePair val = optimizer.optimize(new MaxEval(200),
                new UnivariateObjectiveFunction(f),
                GoalType.MINIMIZE,
                new SearchInterval(0.01, 2));

        return val.getValue();
    }

    private class ErrorFunction implements UnivariateFunction {
        @Override
        public double value(double x) {
            double totalError = 0.0;
            for (int i = 0; i < predictedDistances.size(); i++) {
                double predicted = x * predictedDistances.get(i);
                totalError += squareError(predicted, actualDistances.get(i));
            }
            return totalError;
        }
    }
}
