package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Point3;

public class Field {
    // TODO: FIND THE VERTICAL POSITION OF THE TARGETS

    // X IS FROM BACK TO FRONT AND Y IS FROM LEFT TO RIGHT
    public static final double FIELD_WIDTH = 141.0;
    public static final int NUM_OF_TILES = 6;
    public static final double TILE_WIDTH = FIELD_WIDTH / NUM_OF_TILES; // In millimeters

    public static final double LAUNCH_LINE_X = .5 * TILE_WIDTH;


    // Targets are assumed to be blue and are mirrored as necessary.

    public Point3 STARTER_STACK = new Point3(-.9*TILE_WIDTH, 1.5*TILE_WIDTH, 0);

    public enum Targets {
        // Power shot from left to right
        POWER_SHOT_1(new Point3(3*TILE_WIDTH,0,0)),
        POWER_SHOT_2(new Point3(3*TILE_WIDTH,0,0)),
        POWER_SHOT_3(new Point3(3*TILE_WIDTH,0,0)),

        HIGH_GOAL(new Point3(3*TILE_WIDTH,1.5*TILE_WIDTH,0)),
        MIDDLE_GOAL(new Point3(3*TILE_WIDTH,-1.5*TILE_WIDTH,0)), // On red side
        LOW_GOAL(new Point3(3*TILE_WIDTH,1.5*TILE_WIDTH,0));

        public final Point3 point;

        private Targets(Point3 point) {
            this.point = point;
        }
    }

    public enum TargetZones {
        A(new Point3(0.5*TILE_WIDTH,2.5*TILE_WIDTH,0)),
        B(new Point3(1.5*TILE_WIDTH,1.5*TILE_WIDTH,0)),
        C(new Point3(2.5*TILE_WIDTH,2.5*TILE_WIDTH,0));

        public final Point3 point;

        private TargetZones(Point3 point) {
            this.point = point;
        }
    }
}
