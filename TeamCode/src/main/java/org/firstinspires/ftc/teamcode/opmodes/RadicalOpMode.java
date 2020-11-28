package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Field;

public abstract class RadicalOpMode extends OpMode {
    private static Field.Alliance alliance;
    private static Pose2d lastPose;

    protected Field.Alliance getAlliance() {
        return alliance;
    }
    protected void setAlliance(Field.Alliance alliance) {
        this.alliance = alliance;
    }

    protected Pose2d getLastPose() {
        return lastPose;
    }
    protected void setLastPose(Pose2d lastPose) {
        this.lastPose = lastPose;
    }
}
