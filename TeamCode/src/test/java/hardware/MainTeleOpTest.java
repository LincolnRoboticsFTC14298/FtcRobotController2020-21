package hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine.Collecting;
import org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine.Manual;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mockito;
import org.mockito.runners.MockitoJUnitRunner;

import static com.google.common.truth.Truth.assertThat;
import static org.mockito.Mockito.mock;

@RunWith(MockitoJUnitRunner.class)
public class MainTeleOpTest {

    MainTeleOp teleOp = new MainTeleOp();

    @Before
    public void setUp() {
        teleOp.hardwareMap = mock(HardwareMap.class, Mockito.RETURNS_DEEP_STUBS);

        teleOp.init();
    }

    @Test
    public void ManualStateMachineToCollecting() {
        teleOp.setControlMode(MainTeleOp.ControlMode.FULLY_MANUAL);
        assertThat(teleOp.getNavigationState()).isInstanceOf(Manual.class);

        teleOp.setControlMode(MainTeleOp.ControlMode.MANUAL_COLLECTING);
        assertThat(teleOp.getNavigationState()).isInstanceOf(Collecting.class);

        teleOp.setControlMode(MainTeleOp.ControlMode.FULLY_MANUAL);
        assertThat(teleOp.getNavigationState()).isInstanceOf(Manual.class);

        teleOp.setControlMode(MainTeleOp.ControlMode.FULLY_AUTOMATIC);
        assertThat(teleOp.getNavigationState()).isInstanceOf(Collecting.class);
    }
}
