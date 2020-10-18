package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class ComponentOpMode extends OpMode {
    @Override
    public abstract void init();

    @Override
    public abstract void loop();

    @Override
    public abstract void stop();
}