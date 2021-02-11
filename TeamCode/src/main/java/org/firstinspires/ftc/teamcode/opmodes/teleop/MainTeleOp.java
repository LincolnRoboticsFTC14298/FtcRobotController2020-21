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
    abstract static class NavigationState {
        public void start() {

        }

        public void update() {

        }

        public abstract NavigationState getState();

        public void end() {

        }
    }

    public class Collecting extends NavigationState {

        @Override
        public void update() {
            Field.ringProvider.update(robot.localizer.getPoseEstimate()); // may have to be in main update loop
            switch (controlMode) {
                case AUTOMATIC:
                    if (!robot.drive.isBusy() && Field.ringProvider.getRings().size() > 0) {
                        robot.drive.goToRingAsync();
                    } else if (!robot.drive.isBusy()) {
                        // TODO: rotate to find rings
                    }
                    break;
                case MANUAL:
                    break;
            }
        }

        @Override
        public NavigationState getState() {
            if (getRuntime() > 90 && Field.wobbleGoalProvider.amount() > 0) {
                // Last 30 seconds of match
                return new TravelingToWobbleGoal();
            } else if (robot.ringCounter.getNumberOfRings() == 3) {
                return new TravelingToShoot();
            }
            return this;
        }

        @Override
        public void end() {
            robot.drive.cancelFollowing();
        }
    }

    public class TravelingToShoot extends NavigationState {

        @Override
        public void start() {
            robot.drive.goBehindLineAsync();
        }

        @Override
        public NavigationState getState() {
            if (!robot.drive.isBusy() && robot.drive.isBehindLine()) {
                return new Shooting();
            }
            return this;
        }
    }

    public class Shooting extends NavigationState {

        @Override
        public void start() {
            if (getRuntime() > 90) {
                robot.shootAsync(3);
            } else {
                robot.powerShot();
            }
        }

        @Override
        public NavigationState getState() {
            if (robot.doneShooting()) {
                return new Collecting();
            }
            return this;
        }
    }

    public class TravelingToWobbleGoal extends NavigationState {

        @Override
        public void start() {
            robot.drive.goToWobbleGoalAsync();
        }

        @Override
        public NavigationState getState() {
            if (!robot.drive.isBusy() && robot.drive.isAtWobbleGoal()) {
                Field.wobbleGoalProvider.update(robot.localizer.getPoseEstimate());
                return new TravelingToDropOffWobbleGoal();
            }
            return this;
        }

        @Override
        public void end() {
            robot.arm.closeClaw();
            // May need to sleep or some PICKING_UP state
            robot.arm.lift();
        }
    }

    public class TravelingToDropOffWobbleGoal extends NavigationState {

        @Override
        public void start() {
            robot.drive.goToWallAsync();
        }

        @Override
        public NavigationState getState() {
            if (!robot.drive.isBusy() && robot.drive.isAtWall()) {
                if (Field.wobbleGoalProvider.amount() > 0) {
                    return new TravelingToWobbleGoal();
                } else {
                    robot.arm.closeClaw();
                    return new Collecting();
                }
            }
            return this;
        }

        @Override
        public void end() {
            robot.arm.openClaw();
            // May need to sleep or some IS_DROPPING state
            robot.arm.lower();
        }
    }

    NavigationState navigationState = new Collecting();
    public void updateNavigation() {
        NavigationState newState = navigationState.getState();
        if (navigationState != newState) {
            // Different state //

            // End current state
            navigationState.end();
            // Assign new state
            navigationState = newState;
            // Start the new state
            navigationState.start();
        }
        navigationState.update();
    }

    public void setManualControlMode() {
        if (navigationState instanceof Collecting) {
            robot.drive.cancelFollowing();
        }
        controlMode = ControlMode.MANUAL;
    }
    public void setAutoControlMode() {
        controlMode = ControlMode.AUTOMATIC;
    }
}
