package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Config
public class RingProvider {
    public static double DISTANCE_TILL_DESPAWN_RINGS = 20; // inches

    private List<Ring> rings = new ArrayList<>();

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
    public void addAll(List<Ring> newRings) {
        rings.addAll(newRings);
    }

    public void update(Pose2d pose) {
        for (int i = 0; i < rings.size(); i++) {
            double distance = rings.get(i).distanceFrom(pose.vec());
            if (distance < DISTANCE_TILL_DESPAWN_RINGS) {
                rings.remove(i);
            }
        }
    }

    public Ring getClosest(Vector2d pos) {
        return rings.stream().min(Comparator.comparingDouble((r) -> r.distanceFrom(pos))).get();
    }

    public void draw(Canvas canvas) {
        for (Ring r : rings) {
            r.draw(canvas);
        }
    }
}
