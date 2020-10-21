package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.util.SubsystemOpMode;

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
        setPower(1);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setPower(0);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        robot.subsystemManager.end();
    }

    public void setPower(double n) {
        robot.drive.setPower(n,n,n,n);
        robot.update();
    }
}
