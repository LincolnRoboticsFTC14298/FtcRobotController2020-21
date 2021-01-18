package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotlib.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.robotlib.hardware.gamepad.ButtonTrigger;
import org.firstinspires.ftc.teamcode.robotlib.hardware.gamepad.ButtonType;
import org.firstinspires.ftc.teamcode.robotlib.hardware.gamepad.ToggleButton;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Field.Target;

// Operator Interface
public class OperatorInterface {
    public static final double DEBOUNCER_PERIOD = 0.5;

    private final Robot robot;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    // Use toggle when you want to use the same button for up and down
    ToggleButton autoAimToggleButton, fieldCentricToggleButton, liftArmToggleButton,
            openClawToggleButton, intakeOnToggleButton;

    Button highGoalButton, middleGoalButton, powerShotButton;
    ButtonTrigger shootButton;

    public OperatorInterface(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // ALL OF THE BUTTON MAPPINGS //

        autoAimToggleButton = new ToggleButton(gamepad1, ButtonType.a, 1);
        fieldCentricToggleButton = new ToggleButton(gamepad1, ButtonType.b, 1);
        liftArmToggleButton = new ToggleButton(gamepad1, ButtonType.x);
        openClawToggleButton = new ToggleButton(gamepad1, ButtonType.y);
        intakeOnToggleButton = new ToggleButton(gamepad1, ButtonType.right_bumper);

        highGoalButton = new Button(gamepad2, ButtonType.a);
        middleGoalButton = new Button(gamepad2, ButtonType.b);
        powerShotButton = new Button(gamepad2, ButtonType.x);

        shootButton = new ButtonTrigger(gamepad1, ButtonType.right_trigger);
    }

    public void update() {
        // Robot controlling
        // Targets
        if (highGoalButton.isPressed()) {
            robot.setTarget(Target.HIGH_GOAL);
        }
        if (middleGoalButton.isPressed()) {
            robot.setTarget(Target.MIDDLE_GOAL);
        }

        // Arm
        if (liftArmToggleButton.on()) {
            robot.arm.lift();
        } else if (liftArmToggleButton.off()) {
            robot.arm.lower();
        }
        if (openClawToggleButton.on()) {
            robot.arm.openClaw();
        } else if (openClawToggleButton.off()) {
            robot.arm.closeClaw();
        }

        // Drive
        Pose2d input = getInput();
        robot.drive.teleopControl(input, fieldCentricToggleButton.on(), autoAimToggleButton.on());

        // Intake
        if (intakeOnToggleButton.on()) {
            robot.intake.turnOn();
        } else if (intakeOnToggleButton.off()) {
            robot.intake.turnOff();
        }

        // Shooter + drive
        // THIS IS NOT ASYNC. It will pause what it's doing and aim then shoot.
        if (shootButton.isPressed()) {
            // TODO: if shooting, allow button press to cancel
            // Shoots all three
            robot.shoot(3);
        } else if (powerShotButton.isPressed()) {
            robot.powerShot();
        }
    }

    public Pose2d getInput() {
        return new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad2.right_stick_x);
    }
}
