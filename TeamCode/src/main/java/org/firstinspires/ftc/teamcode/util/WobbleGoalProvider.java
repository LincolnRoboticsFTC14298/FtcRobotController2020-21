package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotlib.util.MathUtil;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.RobotMap.ARM_DOWN_LOCATION_2d;

//@Config
public class WobbleGoalProvider {
    public static double DISTANCE_TILL_DESPAWN_WOBBLE_GOALS = 4; // inches

    private List<WobbleGoal> wobbleGoals = new ArrayList<>();

    public WobbleGoalProvider() {

    }

    public List<WobbleGoal> getWobbleGoals() {
        return wobbleGoals;
    }
    public int amount() {
        return wobbleGoals.size();
    }

    public void add(WobbleGoal wobbleGoal) {
        wobbleGoals.add(wobbleGoal);
    }
    public void addAll(List<Ring> wobbleGoals) {
        wobbleGoals.addAll(wobbleGoals);
    }

    public void update(Pose2d pose) {
        Vector2d armTip = MathUtil.localToGlobal(ARM_DOWN_LOCATION_2d, pose);

        for (int i = 0; i < wobbleGoals.size(); i++) {
            double distance = wobbleGoals.get(i).getPosition().distTo(armTip);
            if (distance < DISTANCE_TILL_DESPAWN_WOBBLE_GOALS) {
                wobbleGoals.remove(i);
            }
        }
    }
    public boolean atWobbleGoal(Pose2d pose) {
        Vector2d armTip = MathUtil.localToGlobal(ARM_DOWN_LOCATION_2d, pose);

        for (int i = 0; i < wobbleGoals.size(); i++) {
            double distance = wobbleGoals.get(i).getPosition().distTo(armTip);
            if (distance < DISTANCE_TILL_DESPAWN_WOBBLE_GOALS) {
                return true;
            }
        }
        return false;
    }

    public WobbleGoal getClosest(Vector2d pos) {
        return wobbleGoals.stream().min(Comparator.comparingDouble((r) -> r.distanceFrom(pos))).get();
    }

    public void draw(Canvas canvas, Field.Alliance alliance) {
        for (WobbleGoal w : wobbleGoals) {
            w.draw(canvas, alliance);
        }
    }
}
