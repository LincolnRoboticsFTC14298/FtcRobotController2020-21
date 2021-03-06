package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.opmodes.DataWriterUtil;

@TeleOp(name="Data Writer test", group="Test")
@Disabled
public class DataWriterTest extends OpMode {
    RadicalGamepad radicalGamepad;
    Pose2d pose2d = new Pose2d(-50.2, 47.2, 1.1);
    boolean finished = false;

    @Override
    public void init() {
        radicalGamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void loop() {
        radicalGamepad.update();

        if (radicalGamepad.a) {
            DataWriterUtil.setLastPose(pose2d);
            finished = true;
            telemetry.addData("Wrote pose", pose2d.toString());
            telemetry.addLine("Please close teleop and reopen, then press b.");
        }
        if (radicalGamepad.b && !finished) {
            Pose2d read = DataWriterUtil.getLastPose();
            telemetry.addData("Read pose", read.toString());
        }

        telemetry.update();
    }
}
