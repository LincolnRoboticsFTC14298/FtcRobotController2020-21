package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import static org.firstinspires.ftc.teamcode.robotlib.util.MathUtil.poseToVector2D;
import static org.firstinspires.ftc.teamcode.util.Field.RING_DIAMETER;

public class Ring {
    private final Vector2D position;

    public Ring(Vector2D position) {
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

    public void draw(Canvas canvas) {
        canvas.setFill("#ffcf40");
        canvas.setStroke("#ffc20d");
        canvas.strokeCircle(position.getX(), position.getY(), RING_DIAMETER);
    }
}
