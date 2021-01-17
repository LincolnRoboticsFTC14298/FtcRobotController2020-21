package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotlib.hardware.Subsystem;

@Config
public class RingCounter extends Subsystem {
    //SENSOR
    public static double ONE_RING = 5; // SENSOR UNITS
    public static double TWO_RINGS = 3;
    public static double THREE_RINGS = 1;

    public RingCounter(HardwareMap hardwareMap) {
        super("Ring Counter");

    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

    }

    public int getNumberOfRings() {
        double distance = 1;
        if (distance <= THREE_RINGS) {
            return 3;
        } else if (distance <= TWO_RINGS) {
            return 2;
        } else if (distance <= ONE_RING) {
            return 1;
        } else {
            return 0;
        }
    }
}
