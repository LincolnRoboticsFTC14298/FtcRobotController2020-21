package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotlib.hardware.gamepad.Button;
import org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonTrigger;
import org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType;
import org.firstinspires.ftc.robotlib.hardware.gamepad.ToggleButton;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmodes.teleop.statemachine.Collecting;

// Operator Interface
public class OperatorInterface {
    private final MainTeleOp teleOp;
    private final Robot robot;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    // Use toggle when you want to use the same button for up and down
    ToggleButton autoAimToggleButton, fieldCentricToggleButton;

    Button fullyManualModeButton, manualCollectingModeButton, fullyAutoModeButton;

    ButtonTrigger cancelShotLeft, cancelShotRight;

    Button highGoalShootingButton, powerShotButton;
    ToggleButton armToggleButton, clawToggleButton, intakeToggleButton;

    public OperatorInterface(MainTeleOp teleOp, Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.teleOp = teleOp;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // ALL OF THE BUTTON MAPPINGS //

        autoAimToggleButton = new ToggleButton(gamepad1, ButtonType.a, 1);
        fieldCentricToggleButton = new ToggleButton(gamepad1, ButtonType.b, 1);

        fullyManualModeButton = new Button(gamepad1, ButtonType.dpad_left, 2);
        manualCollectingModeButton = new Button(gamepad1, ButtonType.dpad_up, 2);
        fullyAutoModeButton = new Button(gamepad1, ButtonType.dpad_right, 2);

        cancelShotLeft = new ButtonTrigger(gamepad1, ButtonType.left_trigger, 1);
        cancelShotRight = new ButtonTrigger(gamepad1, ButtonType.right_trigger, 1);

        // FULLY MANUAL MAPPINGS //
        highGoalShootingButton = new Button(gamepad1, ButtonType.right_bumper, 1);
        powerShotButton = new Button(gamepad1, ButtonType.left_bumper, 1);

        armToggleButton = new ToggleButton(gamepad1, ButtonType.x, 1);
        clawToggleButton = new ToggleButton(gamepad1, ButtonType.y, 1);
        intakeToggleButton = new ToggleButton(gamepad1, ButtonType.right_bumper, 1);
    }

    // Robot controlling //
    public void update() {

        if (fullyManualModeButton.isPressed()) {
            teleOp.setControlMode(MainTeleOp.ControlMode.FULLY_MANUAL);
        } else if (manualCollectingModeButton.isPressed()) {
            teleOp.setControlMode(MainTeleOp.ControlMode.MANUAL_COLLECTING);
        } else if (manualCollectingModeButton.isPressed()) {
            teleOp.setControlMode(MainTeleOp.ControlMode.FULLY_AUTOMATIC);
        }

        if (cancelShotLeft.isPressed() && cancelShotRight.isPressed()) {
            robot.cancelShot();
        }

        Pose2d input = getInput();
        switch (teleOp.getControlMode()) {
            case FULLY_MANUAL:
                // Drive //
                if (robot.doneShooting()) {
                    robot.drive.teleopControl(input, fieldCentricToggleButton.isOn(), autoAimToggleButton.isOn());
                }

                // Shooting //
                if (highGoalShootingButton.isPressed()) {
                    robot.shootAsync(3);
                } else if (powerShotButton.isPressed()) {
                    robot.powerShot();
                }

                // Arm //
                if (armToggleButton.changedToOn()) {
                    robot.arm.lift();
                } else if (armToggleButton.changedToOff()) {
                    robot.arm.lower();
                }

                if (clawToggleButton.changedToOn()) {
                    robot.arm.closeClaw();
                } else if (clawToggleButton.changedToOff()) {
                    robot.arm.openClaw();
                }

                // Intake //
                if (intakeToggleButton.changedToOn()) {
                    robot.intake.turnOn();
                } else if (intakeToggleButton.changedToOff()) {
                    robot.intake.turnOff();
                }
                break;
            case MANUAL_COLLECTING:
                // Drive //
                if (teleOp.getNavigationState() instanceof Collecting && robot.doneShooting()) {
                    robot.drive.teleopControl(input, fieldCentricToggleButton.isOn(), autoAimToggleButton.isOn());
                }
                break;
        }
    }

    public Pose2d getInput() {
        return new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad2.right_stick_x);
    }
}
