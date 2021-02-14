package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.util.statemachine.State;
import org.firstinspires.ftc.robotlib.util.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.DataWriterUtil;
import org.firstinspires.ftc.teamcode.util.Field;

@TeleOp(name="Main TeleOp", group="TeleOp")
@Disabled
public class MainTeleOp extends OpMode {
    private Robot robot;
    private OperatorInterface operatorInterface = new OperatorInterface(this, robot, gamepad1, gamepad2);

    enum ControlMode {
        FULLY_MANUAL,
        MANUAL_COLLECTING,
        FULLY_AUTOMATIC
    }

    ControlMode controlMode = ControlMode.MANUAL_COLLECTING;

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

        switch (controlMode) {
            case FULLY_MANUAL:
                break;
            case MANUAL_COLLECTING:
            case FULLY_AUTOMATIC:
                updateNavigation();
                break;
        }

        robot.update();

        // TODO: Add telemetry
    }

    @Override
    public void stop() {
        robot.stop();
    }

    // Navigation //
    public class Manual extends State {

        @Override
        public State getState() {
            return this;
        }
    }

    public class Collecting extends State {

        @Override
        public void update() {
            Field.ringProvider.update(robot.localizer.getPoseEstimate()); // may have to be in main update loop
            switch (controlMode) {
                case MANUAL_COLLECTING:
                    break;
                case FULLY_AUTOMATIC:
                    if (!robot.drive.isBusy() && Field.ringProvider.getRings().size() > 0) {
                        robot.drive.goToRingAsync();
                    } else if (!robot.drive.isBusy()) {
                        // TODO: rotate to find rings
                    }
                    break;
            }
        }

        @Override
        public State getState() {
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
    
    public class TravelingToShoot extends State {

        @Override
        public void start() {
            robot.drive.goBehindLineAsync();
        }

        @Override
        public State getState() {
            if (!robot.drive.isBusy() && robot.drive.isBehindLine()) {
                return new Shooting();
            }
            return this;
        }
    }

    public class Shooting extends State {

        @Override
        public void start() {
            if (getRuntime() > 90) {
                robot.shootAsync(3);
            } else {
                robot.powerShot();
            }
        }

        @Override
        public State getState() {
            if (robot.doneShooting()) {
                return new Collecting();
            }
            return this;
        }
    }

    public class TravelingToWobbleGoal extends State {

        @Override
        public void start() {
            robot.drive.goToWobbleGoalAsync();
        }

        @Override
        public State getState() {
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

    public class TravelingToDropOffWobbleGoal extends State {

        @Override
        public void start() {
            robot.drive.goToWallAsync();
        }

        @Override
        public State getState() {
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

    StateMachine navigationStateMachine = new StateMachine(new Collecting());
    public void updateNavigation() {
        navigationStateMachine.update();
    }

    public void setControlMode(ControlMode controlMode) {
        switch (controlMode) {
            case FULLY_MANUAL:
                robot.drive.cancelFollowing();
                navigationStateMachine.setState(new Manual());
                break;
            case MANUAL_COLLECTING:
                if (navigationStateMachine.getState() instanceof Manual) {
                    navigationStateMachine.setState(new Collecting());
                } else if (navigationStateMachine.getState() instanceof Collecting) {
                    robot.drive.cancelFollowing();
                }
                break;
            case FULLY_AUTOMATIC:
                if (navigationStateMachine.getState() instanceof Manual) {
                    navigationStateMachine.setState(new Collecting());
                }
                // do nothing
                break;
        }
        this.controlMode = controlMode;
    }
}
