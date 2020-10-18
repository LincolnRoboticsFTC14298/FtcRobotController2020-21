package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.ComponentOpMode;

import static java.lang.Thread.sleep;


@Autonomous(name="Drive to line", group="Autonomous")
public class DriveToLine extends ComponentOpMode {
    Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap, this);
    }

    @Override
    public void start() {
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
        robot.componentManager.end();
    }
}
