package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class WobbleGoal {
    public static final double WOBBLE_GOAL_RADIUS = 6;
    public static final double WOBBLE_GOAL_DIAMETER = 2 * WOBBLE_GOAL_RADIUS;

    private final Vector2d position;

    public WobbleGoal(Vector2d position) {
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

    public void draw(Canvas canvas, Field.Alliance alliance) {
        // TODO: change color
        switch(alliance) {
            case RED:
                canvas.setFill("#b93e40");
                canvas.setStroke("#b93e40");
                break;
            default:
            case BLUE:
                canvas.setFill("#2c5bd3");
                canvas.setStroke("#2c5bd3");
                break;
        }
        canvas.strokeCircle(position.getX(), position.getY(), WOBBLE_GOAL_RADIUS);
    }
}
