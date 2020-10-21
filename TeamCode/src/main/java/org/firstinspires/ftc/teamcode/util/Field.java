package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Field {
    // TODO: FIND THE VERTICAL POSITION OF THE TARGETS
    // NOTE: Maybe we should make our own Point3d?

    // X IS FROM BACK TO FRONT AND Y IS FROM LEFT TO RIGHT
    public static final double FIELD_WIDTH = 141.0;
    public static final int NUM_OF_TILES = 6;
    public static final double TILE_WIDTH = FIELD_WIDTH / NUM_OF_TILES; // In millimeters

    public static final double LAUNCH_LINE_X = .5 * TILE_WIDTH;


    // Targets are assumed to be blue and are mirrored as necessary.

    public Pose2d STARTER_STACK = new Pose2d(-.9*TILE_WIDTH, 1.5*TILE_WIDTH);

    public enum Alliance {
        BLUE,
        RED;
    }

    public enum Target {
        // Power shot from left to right
        OUTWARD_POWER_SHOT(new Vector3D(3*TILE_WIDTH,0,0)),
        MIDDLE_POWER_SHOT(new Vector3D(3*TILE_WIDTH,0,0)),
        INWARD_POWER_SHOT(new Vector3D(3*TILE_WIDTH,0,0)),

        HIGH_GOAL(new Vector3D(3*TILE_WIDTH,1.5*TILE_WIDTH,0)),
        MIDDLE_GOAL(new Vector3D(3*TILE_WIDTH,-1.5*TILE_WIDTH,0)), // On red side
        LOW_GOAL(new Vector3D(3*TILE_WIDTH,1.5*TILE_WIDTH,0));

        private final Vector3D location;

        Target(Vector3D location) {
            this.location = location;
        }

        public Vector3D getLocation(Alliance alliance) {
            if (alliance == Alliance.RED) {
                // Mirror if red
                return mirror(location);
            } else {
                return location;
            }
        }
        public Pose2d getPose(Alliance alliance) {
            Pose2d pose = new Pose2d(location.getX(), location.getY());
            if (alliance == Alliance.RED) {
                // Mirror if red
                return mirror(pose);
            } else {
                return pose;
            }
        }
    }

    public enum TargetZone {
        A(new Pose2d(0.5*TILE_WIDTH,2.5*TILE_WIDTH)),
        B(new Pose2d(1.5*TILE_WIDTH,1.5*TILE_WIDTH)),
        C(new Pose2d(2.5*TILE_WIDTH,2.5*TILE_WIDTH));

        private final Pose2d location;

        TargetZone(Pose2d location) {
            this.location = location;
        }

        public Pose2d getLocation(Alliance alliance) {
            if (alliance == Alliance.RED) {
                return mirror(location);
            } else {
                return location;
            }
        }
    }

    private static Vector3D mirror(Vector3D location) {
        return new Vector3D(location.getX(), -location.getY(), location.getZ());
    }
    private static Pose2d mirror(Pose2d location) {
        return new Pose2d(location.getX(), -location.getY());
    }
}
