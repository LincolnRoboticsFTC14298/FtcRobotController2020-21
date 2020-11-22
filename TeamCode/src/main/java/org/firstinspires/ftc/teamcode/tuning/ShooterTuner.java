package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.optim.univariate.UnivariateOptimizer;
import org.apache.commons.math3.optim.univariate.UnivariatePointValuePair;
import org.firstinspires.ftc.teamcode.hardware.RobotMap;
import org.firstinspires.ftc.teamcode.hardware.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.util.Field;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.SHOOTER_LOCATION;
import static org.firstinspires.ftc.teamcode.util.MathUtil.squareError;

@Config
@TeleOp(name="Shooter", group="Tuner")
public class ShooterTuner extends OpMode {
    public static double angle = 45.0;
    public static double lambda = 9.8 / 50;

    private ArrayList<Double> distances = new ArrayList<>();
    private ArrayList<Double> heights = new ArrayList<>();
    private ArrayList<Double> actuals = new ArrayList<>();

    private Drive drive;
    private Shooter shooter;
    private RadicalGamepad gamepad;

    Field.Target target = Field.Target.HIGH_GOAL;
    Field.Alliance alliance = Field.Alliance.BLUE;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);

        drive.init();
        shooter.init();

        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.update();
        Pose2d input = new Pose2d(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
        drive.teleopControl(input, true, true);

        shooter.setFlapAngle(Math.toRadians(angle));

        if (gamepad.dpad_up) {
            setTarget(Field.Target.HIGH_GOAL);
        }
        if (gamepad.dpad_left) {
            setTarget(Field.Target.MIDDLE_GOAL);
        }
        if (gamepad.dpad_right) {
            setTarget(Field.Target.MIDDLE_POWER_SHOT);
        }

        if (gamepad.left_bumper) {
            alliance = Field.Alliance.BLUE;
        }
        if (gamepad.right_bumper) {
            alliance = Field.Alliance.RED;
        }

        if (gamepad.a) {
            shoot();
        } else if (gamepad.b) {
            distances.add(getDistHeight(drive.getTargetPose())[0]);
            heights.add(getDistHeight(drive.getTargetPose())[1]);
            actuals.add(shooter.getTargetAngle());
            lambda = findLambda();
        }

        telemetry.addData("Angle: ", Math.toDegrees(shooter.getTargetAngle()));
        telemetry.addData("Distance: ", getDistHeight(drive.getTargetPose())[0]);
        telemetry.addData("Height: ", getDistHeight(drive.getTargetPose())[1]);
        telemetry.addData("Lambda: ", lambda);
        telemetry.addLine();
        telemetry.addData("Target: ", target);
        telemetry.addData("Alliance: ", alliance);
    }

    public double[] getDistHeight(Pose2d targetRelPose) {
        double dist = Math.hypot(targetRelPose.getY(), targetRelPose.getX());
        double height = target.getLocation(alliance).getZ();
        return new double[]{dist, height};
    }

    public void setTarget(Field.Target target) {
        this.target = target;
        drive.setTarget(target);
        shooter.setTarget(target); // This is useless
    }

    public void shoot() {
        // Not async as to prevent other movements.
        drive.pointAtTargetAsync();
        ElapsedTime elapsedTime = new ElapsedTime();
        while(drive.isBusy() && elapsedTime.milliseconds() < RobotMap.TIMEOUT) {
            drive.update();
            shooter.update();
        }
        shooter.shoot(drive.getTargetPose());
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
