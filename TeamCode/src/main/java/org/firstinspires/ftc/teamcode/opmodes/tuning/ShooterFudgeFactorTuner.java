package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.util.Field;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotlib.util.MathUtil.squareError;

@Config
@TeleOp(name="Shooter Fudge Factor Tuner", group="Tuner")
@Disabled
public class ShooterFudgeFactorTuner extends OpMode {
    public static double angle = 45.0;
    public static double fudgeFactor = 1;

    private final ArrayList<Double> predictedAngles = new ArrayList<>();
    private final ArrayList<Double> actualAngles = new ArrayList<>();

    private Robot robot;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void start() {
        robot.start();
        robot.setPoseEstimate(new Pose2d(0,0,0));
        Localizer.fudgeFactor = 1;
    }

    @Override
    public void loop() {
        gamepad.update();

        Pose2d input = new Pose2d(-gamepad.left_stick_y, -gamepad.left_stick_x, gamepad.right_stick_x);
        robot.drive.teleopControl(input, true, true);

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

        if (gamepad.left_bumper) {
            robot.setAlliance(Field.Alliance.BLUE);
        }
        if (gamepad.right_bumper) {
            robot.setAlliance(Field.Alliance.RED);
        }

        if (gamepad.a) {
            robot.shootAsync();
        } else if (gamepad.b) {
            predictedAngles.add(robot.localizer.getTargetLaunchAngle());
            actualAngles.add(robot.shooter.getTargetAngle());
            fudgeFactor = findLambda();
        }

        robot.update();

        telemetry.addData("Set Angle: ", Math.toDegrees(robot.shooter.getTargetAngle()));
        telemetry.addData("Predicted Angle: ", Math.toDegrees(fudgeFactor*robot.localizer.getTargetLaunchAngle()));
        telemetry.addData("Pose: ", robot.localizer.getPoseEstimate().toString());
        telemetry.addData("Target: ", robot.getTarget().toString());
        telemetry.addLine();
        telemetry.addData("Fudge Factor: ", fudgeFactor);
        telemetry.addData("Alliance: ", robot.getAlliance());
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

            for (int i = 0; i < predictedAngles.size(); i++) {
                double predicted = x * predictedAngles.get(i);
                totalError += squareError(predicted, actualAngles.get(i));
            }

            return totalError;
        }
    }
}
