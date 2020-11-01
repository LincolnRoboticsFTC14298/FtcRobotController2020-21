package org.firstinspires.ftc.teamcode.hardware.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Subsystem {
    void init(HardwareMap hardwareMap);
    void update();
    void stop();
}
