package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.opmodes.DataWriterUtil;

@TeleOp(name="Data Writer test", group="Test")
public class DataWriterTest extends OpMode {
    RadicalGamepad radicalGamepad;
    Pose2d pose2d = new Pose2d(-50.2, 47.2, 1.1);

    @Override
    public void init() {
        radicalGamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        radicalGamepad.update();

        if (radicalGamepad.a) {
            DataWriterUtil.saveLastPose(pose2d);
            telemetry.addData("Wrote pose", pose2d.toString());
        }
        if (radicalGamepad.b) {
            Pose2d read = DataWriterUtil.readLastPose();
            telemetry.addData("Read pose", read.toString());
        }

        telemetry.update();
    }
}
