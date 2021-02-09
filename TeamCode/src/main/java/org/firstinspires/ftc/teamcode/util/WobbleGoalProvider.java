package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.Comparator;
import java.util.List;

import static org.firstinspires.ftc.robotlib.util.MathUtil.poseToVector2D;
import static org.firstinspires.ftc.robotlib.util.MathUtil.rotateVector;
import static org.firstinspires.ftc.robotlib.util.MathUtil.vector3DToVector2D;
import static org.firstinspires.ftc.teamcode.hardware.RobotMap.ARM_DOWN_LOCATION;

@Config
public class WobbleGoalProvider {
    public static double DISTANCE_TILL_DESPAWN_WOBBLE_GOALS = 4; // inches

    private List<WobbleGoal> wobbleGoals;

    public WobbleGoalProvider() {

    }

    public List<WobbleGoal> getWobbleGoals() {
        return wobbleGoals;
    }

    public void add(WobbleGoal wobbleGoal) {
        wobbleGoals.add(wobbleGoal);
    }
    public void add(List<Ring> wobbleGoals) {
        wobbleGoals.addAll(wobbleGoals);
    }

    public void update(Pose2d pose) {
        for (int i = 0; i < wobbleGoals.size(); i++) {
            Vector2D pos = poseToVector2D(pose);
            Vector2D armTip = pos.add(
                    rotateVector(vector3DToVector2D(ARM_DOWN_LOCATION), pose.getHeading())
            );
            double distance = wobbleGoals.get(i).getPosition().distance(armTip);
            if (distance < DISTANCE_TILL_DESPAWN_WOBBLE_GOALS) {
                wobbleGoals.remove(i);
            }
        }
    }

    public WobbleGoal getClosest(Vector2D pos) {
        return wobbleGoals.stream().min(Comparator.comparingDouble((r) -> r.distanceFrom(pos))).get();
    }
    public WobbleGoal getClosest(Pose2d pose) {
        return getClosest(poseToVector2D(pose));
    }

    public void draw(Canvas canvas, Field.Alliance alliance) {
        for (WobbleGoal w : wobbleGoals) {
            w.draw(canvas, alliance);
        }
    }
}
