package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.Field;

import java.util.prefs.Preferences;

public abstract class DataWriterUtil {
    private static final Preferences prefs = Preferences.userNodeForPackage(DataWriterUtil.class);

    public static Field.Alliance readAlliance() {
        String alliance = prefs.get("alliance", "BLUE");
        return Field.Alliance.valueOf(alliance);
    }
    public static void saveAlliance(Field.Alliance alliance) {
        prefs.put("alliance", alliance.toString());
    }

    public static Pose2d readLastPose() {
        double x = prefs.getDouble("lastPoseX", 0);
        double y = prefs.getDouble("lastPoseY", 0);
        double h = prefs.getDouble("lastPoseHeading", 0);
        return new Pose2d(x, y, h);
    }
    public static void saveLastPose(Pose2d lastPose) {
        double x = lastPose.getX();
        double y = lastPose.getY();
        double h = lastPose.getHeading();

        prefs.putDouble("lastPoseX",       x);
        prefs.putDouble("lastPoseY",       y);
        prefs.putDouble("lastPoseHeading", h);
    }
}
