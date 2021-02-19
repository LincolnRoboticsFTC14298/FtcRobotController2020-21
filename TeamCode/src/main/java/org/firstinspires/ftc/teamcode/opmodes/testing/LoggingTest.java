package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.google.common.flogger.FluentLogger;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LoggingTest extends OpMode {
    private static FluentLogger logger = FluentLogger.forEnclosingClass();

    private Pose2d testPose = new Pose2d(15,-20, Math.toRadians(45));

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        logger.atInfo().log("Pose: %s", testPose);
        logger.atInfo().log("Info");
        logger.atWarning().log("Warning");
        logger.atSevere().log("Severe");
        logger.atFine().log("Fine");
        logger.atFiner().log("Finer");
        logger.atFinest().log("Finest");
        logger.atConfig().log("Config");
    }
}
