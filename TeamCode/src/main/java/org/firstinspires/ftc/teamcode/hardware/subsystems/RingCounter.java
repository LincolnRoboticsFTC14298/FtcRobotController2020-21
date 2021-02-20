package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;
import org.firstinspires.ftc.robotlib.util.MathUtil;

import static org.firstinspires.ftc.teamcode.util.Ring.RING_RADIUS;

@Config
public class RingCounter extends AbstractSubsystem {
    private static FluentLogger logger = FluentLogger.forEnclosingClass();
    //TODO: ADD FRONT AND CARTRIDGE SENSOR
    public static double wallError = .2; // in
    public static double ringError = .5;
    public static double wallDist = 4;

    public static double ONE_RING = 5; // SENSOR UNITS
    public static double TWO_RINGS = 3;
    public static double THREE_RINGS = 1;

    private double cartridgeDist;
    private double lastDist = wallDist; // Default, wall length
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
        currDist = 0; // read sensor
        cartridgeDist = 0;
    }

    @Override
    public void update() {
        // INTAKE SENSOR //
        dx = Math.abs(currDist - lastDist);
        if (!reversed && (wallToRing() || ringToRing())) {
            totalRings++;
        } else if (reversed && (ringToRing() || ringToWall())) {
            totalRings--;
        }
        lastDist = currDist;

        // CARTRIDGE //
        if (cartridgeDist <= THREE_RINGS) {
            numOfRingsCartridge = 3;
        } else if (cartridgeDist <= TWO_RINGS) {
            numOfRingsCartridge = 2;
        } else if (cartridgeDist <= ONE_RING) {
            numOfRingsCartridge = 1;
        } else {
            numOfRingsCartridge = 0;
        }
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
        telemetry.put("Cartridge Dist", cartridgeDist);
    }

    @Override
    public void updateLogging() {
        logger.atInfo().log("Total Cartridge Cartridge: %f", numOfRingsCartridge);
        logger.atInfo().log("Total: %f", totalRings);
        logger.atInfo().log("Reversed: %f", reversed);

        logger.atInfo().log("Cartridge distance: %f", cartridgeDist);
        logger.atInfo().log("Sensor distance: %f", currDist);
    }

    private boolean rawChange() {
        // Check if change in distance of the sensor is greater than a rings radius
        return dx > RING_RADIUS - ringError;
    }
    private static boolean atWall(double dist) {
        return MathUtil.differenceWithinError(dist, wallDist, wallError);
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
