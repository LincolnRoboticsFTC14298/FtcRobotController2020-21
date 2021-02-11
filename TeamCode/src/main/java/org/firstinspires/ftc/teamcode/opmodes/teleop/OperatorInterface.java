package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonTrigger;
import org.firstinspires.ftc.robotlib.hardware.gamepad.ButtonType;
import org.firstinspires.ftc.robotlib.hardware.gamepad.ToggleButton;
import org.firstinspires.ftc.teamcode.hardware.Robot;

// Operator Interface
public class OperatorInterface {
    public static final double DEBOUNCER_PERIOD = 0.5;

    private final MainTeleOp teleOp;
    private final Robot robot;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    // Use toggle when you want to use the same button for up and down
    ToggleButton autoAimToggleButton, fieldCentricToggleButton, manualModeToggleButton;

    ButtonTrigger cancelShotLeft, cancelShotRight;

    public OperatorInterface(MainTeleOp teleOp, Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.teleOp = teleOp;
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // ALL OF THE BUTTON MAPPINGS //

        autoAimToggleButton = new ToggleButton(gamepad1, ButtonType.a, 1);
        fieldCentricToggleButton = new ToggleButton(gamepad1, ButtonType.b, 1);
        manualModeToggleButton = new ToggleButton(gamepad1, ButtonType.x, 1);

        cancelShotLeft = new ButtonTrigger(gamepad1, ButtonType.left_trigger, 1);
        cancelShotRight = new ButtonTrigger(gamepad1, ButtonType.right_trigger, 1);
    }

    // Robot controlling //
    public void update() {
        // Drive
        Pose2d input = getInput();
        robot.drive.teleopControl(input, fieldCentricToggleButton.on(), autoAimToggleButton.on());

        if (manualModeToggleButton.changedToOn()) {
            teleOp.setManualControlMode();
        } else if (manualModeToggleButton.changedToOff()) {
            teleOp.setAutoControlMode();
        }

        if (cancelShotLeft.isPressed() && cancelShotRight.isPressed()) {
            robot.cancelShot();
        }

        // TODO: Add some fully manual state just in case
    }

    public Pose2d getInput() {
        return new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad2.right_stick_x);
    }
}
