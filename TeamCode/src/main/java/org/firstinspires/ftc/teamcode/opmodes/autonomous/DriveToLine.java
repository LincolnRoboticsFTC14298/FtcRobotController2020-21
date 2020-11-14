package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import static java.lang.Thread.sleep;


@Autonomous(name="Drive to line", group="Autonomous")
public class DriveToLine extends OpMode {
    Robot robot = new Robot(this);

    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void start() {
        robot.vision.startStreaming();
        robot.vision.analyze();
        robot.vision.stopStreaming();
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
        robot.stop();
    }

    public void setPower(double n) {
        robot.drive.setMotorPowers(n,n,n,n);
        robot.update();
    }
}
