package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;
import org.firstinspires.ftc.robotlib.util.MathUtil;

import java.util.LinkedList;

import static org.firstinspires.ftc.teamcode.util.Ring.RING_RADIUS;

@Config
public class RingCounter extends AbstractSubsystem {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    private static final String DISTANCE_SENSOR_NAME = "distanceSensor";
    private static final String COLOR_SENSOR_NAME = "colorSensor";

    public static double WALL_DIST_MIN_ERROR = .2; // in
    public static double RING_DIST_MIN_ERROR = .5;
    public static double WALL_DIST = 4;

    public static Vector3D RING_COLOR = new Vector3D(0,0,0); // HSV
    public static double RING_COLOR_MIN_ERROR = 1;

    public static double TIME_FROM_FRONT_TO_CARTRIDGE = 1000; // ms

    private LinkedList<ElapsedTime> rings = new LinkedList<>();

    private double lastDist = WALL_DIST; // Default, wall length
    private double currDist;
    private double dx;

    private double[] hsvValues = new double[3];

    private int numOfRingsCartridge; // An approximation
    private int totalRings;

    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;

    private boolean reversed = false;

    public RingCounter(HardwareMap hardwareMap) {
        super("Ring Counter");
        distanceSensor = hardwareMap.get(DistanceSensor.class, DISTANCE_SENSOR_NAME);
        colorSensor = hardwareMap.get(ColorSensor.class, COLOR_SENSOR_NAME);
    }

    @Override
    public void init() {
        colorSensor.enableLed(true);
    }

    @Override
    public void readSensorValues() {
        currDist = distanceSensor.getDistance(DistanceUnit.INCH); // read sensor

        float[] hsv = new float[3];
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);
        for (int i = 0; i < hsvValues.length; i++) {
            hsvValues[i] = hsv[i];
        }
    }

    @Override
    public void update() {
        updateFront();
        updateCartridge();
    }

    @Override
    public void stop() {

    }

    @Override
    public void updateTelemetry() {
        telemetry.put("Total Cartridge Cartridge", numOfRingsCartridge);
        telemetry.put("Total", totalRings);
        telemetry.put("Reversed", reversed);
        telemetry.put("Curr Dist", currDist);
        telemetry.put("Color", hsvValues);
    }

    @Override
    public void updateLogging() {
        logger.atInfo().log("Total Cartridge Cartridge: %f", numOfRingsCartridge);
        logger.atInfo().log("Total: %f", totalRings);
        logger.atInfo().log("Reversed: %f", reversed);
        logger.atInfo().log("Sensor distance: %f", currDist);
        logger.atInfo().log("Color reading: %f", hsvValues);
    }

    private void updateFront() {
        dx = Math.abs(currDist - lastDist);
        if (!reversed && (wallToRing() || ringToRing())) {
            totalRings++;
            rings.add(new ElapsedTime());
        } else if (reversed && (ringToRing() || ringToWall())) {
            // check to see behavior if ring just left but still somewhat in the intake, mainly ring to ring
            totalRings--;
            rings.removeLast();
        }
        lastDist = currDist;
    }

    private void updateCartridge() {
        // In order from oldest to youngest
        if (isCartridgeFull() && numOfRingsCartridge < 3) {
            for (int i = 0; i < 3-numOfRingsCartridge; i++) {
                rings.removeFirst();
            }
            numOfRingsCartridge = 3;
        }

        // Would break if forward -> reverse -> forward, may not be a problem
        while (rings.getFirst().milliseconds() >= TIME_FROM_FRONT_TO_CARTRIDGE) {
            rings.removeFirst();
            numOfRingsCartridge++;
        }
    }

    private boolean rawChange() {
        // Check if change in distance of the sensor is greater than a rings radius
        return dx > RING_RADIUS - RING_DIST_MIN_ERROR;
    }
    private static boolean atWall(double dist) {
        return MathUtil.differenceWithinError(dist, WALL_DIST, WALL_DIST_MIN_ERROR);
    }

    private boolean wallToRing() {
        return rawChange() && atWall(lastDist);
    }
    private boolean ringToWall() {
        return rawChange() && atWall(currDist);
    }
    private boolean ringToRing() {
        return rawChange() && !atWall(lastDist) && !atWall(currDist);
    }

    public int getNumberOfRingsInCartridge() {
        return numOfRingsCartridge;
    }
    public int getTotalRings() {
        return totalRings;
    }
    public boolean allRingsInCartridge() {
        return totalRings == numOfRingsCartridge;
    }

    public boolean isCartridgeFull() {
        double d = Vector3D.distance(RING_COLOR, new Vector3D(hsvValues));
        return MathUtil.differenceWithinError(d, 0, RING_COLOR_MIN_ERROR);
    }

    public void setReversed(boolean reversed) {
        this.reversed = reversed;
    }

    public void removeRingFromIntake() {
        totalRings--;
    }
    public void removeRingFromCartridge() {
        totalRings--;
        numOfRingsCartridge--;
    }
}
