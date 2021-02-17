package org.firstinspires.ftc.robotlib.hardware.mocks;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareMapMock extends HardwareMap {
    public HardwareMapMock() {
        super(null);
    }

    @Deprecated
    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        try {
            return classOrInterface.newInstance();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (InstantiationException e) {
            e.printStackTrace();
        }
        return null;
    }
}
