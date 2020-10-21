package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.SubsystemOpMode;

import static java.lang.Thread.sleep;


@Autonomous(name="Drive to line", group="Autonomous")
public class DriveToLine extends SubsystemOpMode {
    Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(this);
    }

    @Override
    public void start() {
        robot.vision.analyze();
        robot.drive.setPower(1,1,1,1);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.drive.setPower(0,0,0,0);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        robot.subsystemManager.end();
    }
}
