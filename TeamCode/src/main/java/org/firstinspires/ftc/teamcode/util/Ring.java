package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Ring {
    public static final double RING_RADIUS = 5.0; // inches
    public static final double RING_DIAMETER = 2 * RING_RADIUS; // inches

    private final Vector2d position;

    public Ring(Vector2d position) {
        this.position = position;
    }

    public double distanceFrom(Vector2d pos) {
        return position.distTo(pos);
    }
    public double distanceFrom(Ring ring) {
        return position.distTo(ring.getPosition());
    }

    public Vector2d getPosition() {
        return position;
    }

    public void draw(Canvas canvas) {
        canvas.setFill("#ffcf40");
        canvas.setStroke("#ffc20d");
        canvas.strokeCircle(position.getX(), position.getY(), RING_RADIUS);
    }
}
