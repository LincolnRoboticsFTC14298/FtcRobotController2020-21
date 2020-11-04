package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.hardware.util.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.util.Field;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.SHOOTER_LOCATION;
import static org.firstinspires.ftc.teamcode.util.MathUtil.squareError;

@Config
@TeleOp(name="Shooter", group="Tuner")
public class ShooterTuner extends OpMode {
    public static double angle = 45.0;
    public static double lambda = 9.8 / 50;

    ArrayList<Double> distances = new ArrayList<>();
    ArrayList<Double> heights = new ArrayList<>();
    ArrayList<Double> actuals = new ArrayList<>();

    Robot robot = new Robot();
    RadicalGamepad gamepad;

    @Override
    public void init() {
        robot.init(this);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.update();
        Pose2d input = new Pose2d(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.left_stick_x);
        robot.drive.teleopControl(input, false, true);

        robot.shooter.setFlapAngle(Math.toRadians(angle));

        if (gamepad.dpad_up) {
            robot.setTarget(Field.Target.HIGH_GOAL);
        }
        if (gamepad.dpad_left) {
            robot.setTarget(Field.Target.MIDDLE_GOAL);
        }
        if (gamepad.dpad_right) {
            robot.setTarget(Field.Target.MIDDLE_POWER_SHOT);
        }

        if (gamepad.a) {
            robot.shoot(1);
        } else if (gamepad.b) {
            distances.add(getDistHeight(robot.drive.getTargetPose())[0]);
            heights.add(getDistHeight(robot.drive.getTargetPose())[1]);
            actuals.add(robot.shooter.getTargetAngle());
            lambda = findLambda();
        }

        telemetry.addData("Angle: ", Math.toDegrees(robot.shooter.getTargetAngle()));
        telemetry.addData("Distance: ", getDistHeight(robot.drive.getTargetPose())[0]);
        telemetry.addData("Height: ", getDistHeight(robot.drive.getTargetPose())[0]);
        telemetry.addData("Lambda: ", lambda);
    }

    public double[] getDistHeight(Pose2d targetRelPose) {
        double dist = Math.hypot(targetRelPose.getY(), targetRelPose.getX());
        double height = robot.getTarget().getLocation(robot.getAlliance()).getZ();
        return new double[]{dist, height};
    }

    public double findLambda() {
        UnivariateFunction f = new ErrorFunction();
        UnivariateOptimizer optimizer = new BrentOptimizer(1e-10, 1e-14);

        UnivariatePointValuePair val = optimizer.optimize(new MaxEval(200),
                new UnivariateObjectiveFunction(f),
                GoalType.MINIMIZE,
                new SearchInterval(0, 2));

        return val.getValue();
    }

    private class ErrorFunction implements UnivariateFunction {
        @Override
        public double value(double x) {
            double totalError = 0.0;
            for (int i = 0; i < distances.size(); i++) {
                double d = distances.get(i), h = heights.get(i);
                double k = 1.0 / (2 * d * x);
                double det = 1 - 4 * (h - SHOOTER_LOCATION.getZ() + x * d * d) * x;
                double predicted = Math.atan(k - k * Math.sqrt(det));
                totalError += squareError(predicted, actuals.get(i));
            }
            return totalError;
        }
    }
}
