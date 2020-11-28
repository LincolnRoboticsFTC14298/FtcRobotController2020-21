package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.Field;

import java.io.FileReader;
import java.io.FileWriter;
import java.util.Properties;

public abstract class DataWriterUtil {
    private static final String FILE_NAME = "data.properties";

    public static Field.Alliance getAlliance() {
        Properties p = getProperty();
        return Field.Alliance.valueOf(p.getProperty("alliance"));
    }
    public static void setAlliance(Field.Alliance alliance) {
        Properties p = getProperty();
        p.setProperty("alliance", String.valueOf(alliance));
        store(p);
    }

    public static Pose2d getLastPose() {
        Properties p = getProperty();
        double x = Double.parseDouble(p.getProperty("lastPoseX"));
        double y = Double.parseDouble(p.getProperty("lastPoseY"));
        double h = Double.parseDouble(p.getProperty("lastPoseHeading"));
        return new Pose2d(x, y, h);
    }
    public static void setLastPose(Pose2d lastPose) {
        double x = lastPose.getX();
        double y = lastPose.getY();
        double h = lastPose.getHeading();

        Properties p = getProperty();
        p.setProperty("lastPoseX", String.valueOf(x));
        p.setProperty("lastPoseY", String.valueOf(y));
        p.setProperty("lastPoseHeading", String.valueOf(h));
        store(p);
    }

    private static Properties getProperty() {
        try {
            // create a reader object on the properties file
            FileReader reader = new FileReader(FILE_NAME);

            // create properties object
            Properties p = new Properties();

            // Add a wrapper around reader object
            p.load(reader);

            return p;
        } catch (Exception e) {
            return null;
        }
    }
    private static void store(Properties p) {
        try {
            p.store(new FileWriter(FILE_NAME), "Latest data");
        } catch (Exception e) {

        }
    }
}
