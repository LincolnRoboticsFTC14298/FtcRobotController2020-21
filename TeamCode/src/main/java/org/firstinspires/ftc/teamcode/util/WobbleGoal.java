package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import static org.firstinspires.ftc.robotlib.util.MathUtil.poseToVector2D;
import static org.firstinspires.ftc.teamcode.util.Field.RING_DIAMETER;

public class WobbleGoal {
    private final Vector2D position;

    public WobbleGoal(Vector2D position) {
        this.position = position;
    }

    public double distanceFrom(Vector2D pos) {
        return position.distance(pos);
    }
    public double distanceFrom(Pose2d pose) {
        return distanceFrom(poseToVector2D(pose));
    }
    public double distanceFrom(Ring ring) {
        return position.distance(ring.getPosition());
    }

    public Vector2D getPosition() {
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

        canvas.strokeCircle(position.getX(), position.getY(), RING_DIAMETER);
    }
}
