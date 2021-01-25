package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.Field;

public abstract class DataWriterUtil {
    private static Field.Alliance alliance;
    private static Pose2d lastPose;

    public static Field.Alliance getAlliance() {
        return alliance;
    }
    public static void setAlliance(Field.Alliance _alliance) {
        alliance = _alliance;
    }

    public static Pose2d getLastPose() {
        return lastPose;
    }
    public static void setLastPose(Pose2d _lastPose) {
        lastPose = _lastPose;
    }
}
