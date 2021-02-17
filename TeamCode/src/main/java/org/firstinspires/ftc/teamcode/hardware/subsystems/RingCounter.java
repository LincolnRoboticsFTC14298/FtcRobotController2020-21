package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotlib.hardware.AbstractSubsystem;

@Config
public class RingCounter extends AbstractSubsystem {
    //TODO: ADD SENSOR
    public static double ONE_RING = 5; // SENSOR UNITS
    public static double TWO_RINGS = 3;
    public static double THREE_RINGS = 1;

    private int numOfRings;

    public RingCounter(HardwareMap hardwareMap) {
        super("Ring Counter");

    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void updateTelemetry() {
        telemetry.put("Last Reading", numOfRings);
    }

    public int getNumberOfRings() {
        // Only reads when called
        double distance = 1;
        if (distance <= THREE_RINGS) {
            numOfRings = 3;
        } else if (distance <= TWO_RINGS) {
            numOfRings = 2;
        } else if (distance <= ONE_RING) {
            numOfRings = 1;
        } else {
            numOfRings = 0;
        }
        return numOfRings;
    }
}
