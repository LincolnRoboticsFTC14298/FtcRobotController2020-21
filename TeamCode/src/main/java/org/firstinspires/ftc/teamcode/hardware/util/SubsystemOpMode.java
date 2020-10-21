package org.firstinspires.ftc.teamcode.hardware.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class SubsystemOpMode extends OpMode {
    @Override
    public abstract void init();

    @Override
    public abstract void loop();

    @Override
    public abstract void stop();
}