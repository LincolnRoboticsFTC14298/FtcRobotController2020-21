package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.DataWriterUtil;
import org.firstinspires.ftc.teamcode.util.Field;

@TeleOp(name="Main TeleOp", group="TeleOp")
@Disabled
public class MainTeleOp extends OpMode {
    private Robot robot;
    private OperatorInterface operatorInterface = new OperatorInterface(this, robot, gamepad1, gamepad2);

    enum ControlMode {
        MANUAL,
        AUTOMATIC
    }

    ControlMode controlMode = ControlMode.MANUAL;

    @Override
    public void init() {
        robot = new Robot(this);

        robot.setAlliance(DataWriterUtil.getAlliance());
        robot.setPoseEstimate(DataWriterUtil.getLastPose());

        robot.init();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        operatorInterface.update();
        updateNavigation();
        robot.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    // Navigation //
    public enum NavigationStatus {
        COLLECTING,
        TRAVELING_TO_SHOOT,
        SHOOTING,
        TRAVELING_TO_WOBBLE_GOAL,
        TRAVELING_TO_DROP_OFF
    }

    NavigationStatus navigationStatus = NavigationStatus.COLLECTING;
    public void updateNavigation() {
        robot.vision.scan();
        switch (navigationStatus) {
            case COLLECTING:
                if (getRuntime() > 90 && Field.wobbleGoalProvider.amount() > 0) {
                    // Last 30 seconds of match
                    robot.arm.lower();
                    robot.drive.cancelFollowing();
                    navigationStatus = NavigationStatus.TRAVELING_TO_WOBBLE_GOAL;
                } else if (robot.ringCounter.getNumberOfRings() == 3) {
                    robot.drive.cancelFollowing();
                    navigationStatus = NavigationStatus.TRAVELING_TO_SHOOT;
                }
                switch (controlMode) {
                    case AUTOMATIC:
                        Field.ringProvider.update(robot.localizer.getPoseEstimate()); // may have to be in main update loop
                        if (!robot.drive.isBusy() && Field.ringProvider.getRings().size() > 0) {
                            robot.drive.goToRingAsync();
                        } else if (!robot.drive.isBusy()) {
                            // TODO: rotate to find rings
                        }
                        break;
                    case MANUAL:
                        break;
                }
                break;
            case TRAVELING_TO_SHOOT:
                if (!robot.drive.isBusy() && robot.drive.isBehindLine()) {
                    navigationStatus = NavigationStatus.SHOOTING;
                    if (getRuntime() > 90) {
                        robot.shootAsync(3);
                    } else {
                        robot.powerShot();
                    }
                } else if (!robot.drive.isBusy()) {
                    robot.drive.goBehindLineAsync();
                }
                break;
            case SHOOTING:
                if (robot.doneShooting()) {
                    navigationStatus = NavigationStatus.COLLECTING;
                }
                break;
            case TRAVELING_TO_WOBBLE_GOAL:
                if (!robot.drive.isBusy() && robot.drive.isAtWobbleGoal()) {
                    Field.wobbleGoalProvider.update(robot.localizer.getPoseEstimate());
                    robot.arm.closeClaw();
                    // May need to sleep or some PICKING_UP state
                    robot.arm.lift();
                    navigationStatus = NavigationStatus.TRAVELING_TO_DROP_OFF;
                } else if (!robot.drive.isBusy()) {
                    robot.drive.goToWobbleGoalAsync();
                }
                break;
            case TRAVELING_TO_DROP_OFF:
                if (!robot.drive.isBusy() && robot.drive.isAtWall()) {
                    robot.arm.openClaw();
                    // May need to sleep or some IS_DROPPING state
                    robot.arm.lower();
                    if (Field.wobbleGoalProvider.amount() > 0) {
                        navigationStatus = NavigationStatus.TRAVELING_TO_WOBBLE_GOAL;
                    } else {
                        robot.arm.closeClaw();
                        navigationStatus = NavigationStatus.COLLECTING;
                    }
                } else if (!robot.drive.isBusy()) {
                    robot.drive.goToWallAsync();
                }
                break;
        }
    }

    public void setManualMode() {
        switch (navigationStatus) {
            case COLLECTING:
                robot.drive.cancelFollowing();
                break;
        }
        controlMode = ControlMode.MANUAL;
    }
    public void setAutoMode() {
        controlMode = ControlMode.AUTOMATIC;
    }
}
