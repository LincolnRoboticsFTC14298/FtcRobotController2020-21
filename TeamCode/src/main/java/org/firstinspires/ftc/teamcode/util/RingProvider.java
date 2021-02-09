package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.Comparator;
import java.util.List;

import static org.firstinspires.ftc.robotlib.util.MathUtil.poseToVector2D;

@Config
public class RingProvider {
    public static double DISTANCE_TILL_DESPAWN_RINGS = 20; // inches

    private List<Ring> rings;

    public RingProvider() {

    }

    // Doesn't care about overlap
    // TODO: Create statistical filter
    public List<Ring> getRings() {
        return rings;
    }

    public void add(Ring ring) {
        rings.add(ring);
    }
    public void add(List<Ring> newRings) {
        rings.addAll(newRings);
    }

    public void update(Pose2d pose) {
        for (int i = 0; i < rings.size(); i++) {
            double distance = rings.get(i).distanceFrom(pose);
            if (distance < DISTANCE_TILL_DESPAWN_RINGS) {
                rings.remove(i);
            }
        }
    }

    public Ring getClosest(Vector2D pos) {
        return rings.stream().min(Comparator.comparingDouble((r) -> r.distanceFrom(pos))).get();
    }
    public Ring getClosest(Pose2d pose) {
        return getClosest(poseToVector2D(pose));
    }

    public void draw(Canvas canvas) {
        for (Ring r : rings) {
            r.draw(canvas);
        }
    }
}
