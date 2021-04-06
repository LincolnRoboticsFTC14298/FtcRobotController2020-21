package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;
import org.firstinspires.ftc.robotlib.util.MathUtil;

import static org.firstinspires.ftc.teamcode.util.Ring.RING_RADIUS;

@Config
public class RingCounter extends AbstractSubsystem {
    private static final FluentLogger logger = FluentLogger.forEnclosingClass();

    //TODO: ADD FRONT AND CARTRIDGE SENSOR
    public static double WALL_DIST_MIN_ERROR = .2; // in
    public static double RING_DIST_MIN_ERROR = .5;
    public static double WALL_DIST = 4;

    public static double TIME_FROM_FRONT_TO_CARTRIDGE = 1000; // ms

    private LinkedList<ElapsedTime> rings = new LinkedList<>();

    private double lastDist = WALL_DIST; // Default, wall length
    private double currDist;
    private double dx;

    private int numOfRingsCartridge;
    private int totalRings;

    private boolean reversed = false;

    public RingCounter(HardwareMap hardwareMap) {
        super("Ring Counter");

    }

    @Override
    public void readSensorValues() {
        currDist = distanceSensor.getDistance(DistanceUnit.INCH); // read sensor
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
    }

    @Override
    public void updateLogging() {
        logger.atInfo().log("Total Cartridge Cartridge: %f", numOfRingsCartridge);
        logger.atInfo().log("Total: %f", totalRings);
        logger.atInfo().log("Reversed: %f", reversed);
        logger.atInfo().log("Sensor distance: %f", currDist);
    }

    private void updateFront() {
        dx = Math.abs(currDist - lastDist);
        if (!reversed && (wallToRing() || ringToRing())) {
            totalRings++;
            rings.add(new ElapsedTime());
        } else if (reversed && (ringToRing() || ringToWall())) {
            // TODO: check to see behavior if ring just left but still somewhat in the intake, mainly ring to ring
            totalRings--;
            rings.removeLast();
        }
        lastDist = currDist;
    }

    private void updateCartridge() {
        // In order from oldest to youngest
        // TODO: Would break if forward -> reverse -> forward, may not be a problem
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
