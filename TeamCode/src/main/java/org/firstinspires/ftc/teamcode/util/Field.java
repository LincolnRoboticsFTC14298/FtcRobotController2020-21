package org.firstinspires.ftc.teamcode.util;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;


@Config
public class Field {
    // TODO: FIND THE VERTICAL POSITION OF THE TARGETS
    public static final double RING_RADIUS = 5.0; // inches
    public static final double RING_DIAMETER = 2 * RING_RADIUS; // inches

    // X IS FROM BACK TO FRONT AND Y IS FROM LEFT TO RIGHT
    public static final double FIELD_WIDTH = 141.0;
    public static final int NUM_OF_TILES = 6;
    public static final double TILE_WIDTH = FIELD_WIDTH / NUM_OF_TILES; // In inches

    public static final double LAUNCH_LINE_X = .5 * TILE_WIDTH;

    public static RingProvider ringProvider = new RingProvider();
    public static WobbleGoalProvider wobbleGoalProvider = new WobbleGoalProvider();

    // Targets are assumed to be blue and are mirrored as necessary.

    public Vector2D STARTER_STACK = new Vector2D(-.9*TILE_WIDTH, 1.5*TILE_WIDTH);

    public enum Alliance {
        BLUE,
        RED
    }

    public enum Target {
        // Power shot from left to right
        // TODO: FIND Y AND Z
        OUTWARD_POWER_SHOT(new Vector3D(3*TILE_WIDTH,0,30.5)),
        MIDDLE_POWER_SHOT(new Vector3D(3*TILE_WIDTH,0,30.5)),
        INWARD_POWER_SHOT(new Vector3D(3*TILE_WIDTH,0,30.5)),

        // TODO: FIND Z
        HIGH_GOAL(new Vector3D(3*TILE_WIDTH,1.5*TILE_WIDTH,0)),
        MIDDLE_GOAL(new Vector3D(3*TILE_WIDTH,-1.5*TILE_WIDTH,0)), // Blue goal on red side
        LOW_GOAL(new Vector3D(3*TILE_WIDTH,1.5*TILE_WIDTH,0));

        private final Vector3D location;

        Target(Vector3D location) {
            this.location = location;
        }

        public Vector3D getLocation(Alliance alliance) {
            switch(alliance) {
                case RED:
                    return mirror(location);
                default:
                    return location;
            }
        }
    }

    public enum TargetZone {
        A(new Vector2D(0.5*TILE_WIDTH,2.5*TILE_WIDTH)),
        B(new Vector2D(1.5*TILE_WIDTH,1.5*TILE_WIDTH)),
        C(new Vector2D(2.5*TILE_WIDTH,2.5*TILE_WIDTH));

        private final Vector2D location;

        TargetZone(Vector2D location) {
            this.location = location;
        }

        public Vector2D getLocation(Alliance alliance) {
            switch(alliance) {
                case RED:
                    return mirror(location);
                default:
                    return location;
            }
        }
    }

    private static Vector3D mirror(Vector3D location) {
        return new Vector3D(location.getX(), -location.getY(), location.getZ());
    }
    private static Vector2D mirror(Vector2D location) {
        return new Vector2D(location.getX(), -location.getY());
    }

    /*
     * Assumes pose is blue by default
     */
    public static Pose2d conditionalMirror(Pose2d pose, Alliance alliance) {
        switch(alliance) {
            case RED:
                return new Pose2d(pose.getX(), -pose.getY(), -pose.getHeading());
            default:
                return pose;
        }
    }
}
