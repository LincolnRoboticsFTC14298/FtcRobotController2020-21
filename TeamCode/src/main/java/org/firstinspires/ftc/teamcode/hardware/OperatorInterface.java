package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.ButtonType;
import org.firstinspires.ftc.teamcode.util.Field.*;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.util.ToggleButton;

// Operator Interface
public class OperatorInterface {
    private Robot robot;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    // Use toggle when you want to use the same button for up and down
    ToggleButton autoAimToggleButton, localControlToggleButton, liftArmToggleButton, openClawToggleButton, intakeOnToggleButton;

    Button highGoalButton, middleGoalButton, outwardPowerShotButton, middlePowerShotButton, inwardPowerShotButton, powerShotButton;
    Button shootButton;

    // Toggles
    double latestAutoAim = 0, latestIntakeOn = 0, latestLocalControl = 0;

    public OperatorInterface(Robot robot) {
        this.robot = robot;
    }

    public void init(OpMode opMode) {
        // this.gamepad1 = new StickyGamepad(opMode.gamepad1);
        // this.gamepad2 = new StickyGamepad(opMode.gamepad2);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        autoAimToggleButton = new ToggleButton(gamepad1, ButtonType.a, 1);
        localControlToggleButton = new ToggleButton(gamepad1, ButtonType.b, 1);
        liftArmToggleButton = new ToggleButton(gamepad1, ButtonType.x);
        openClawToggleButton = new ToggleButton(gamepad1, ButtonType.y);
        intakeOnToggleButton = new ToggleButton(gamepad1, ButtonType.right_bumper);


        highGoalButton = new Button(gamepad2, ButtonType.a);
        middleGoalButton = new Button(gamepad2, ButtonType.b);

        powerShotButton = new Button(gamepad2, ButtonType.x);

//        outwardPowerShotButton = new Button(gamepad2, ButtonType.dpad_left);
//        middlePowerShotButton = new Button(gamepad2, ButtonType.dpad_up);
//        inwardPowerShotButton = new Button(gamepad2, ButtonType.dpad_right);

        shootButton = new Button(gamepad1, ButtonType.left_bumper);
    }

    public void update() {
        // Update gamepads
        //gamepad1.update();
        //gamepad2.update();

        // Robot controlling
        // Targets
        if (highGoalButton.isPressed()) {
            robot.setTarget(Target.HIGH_GOAL);
        }
        if (middleGoalButton.isPressed()) {
            robot.setTarget(Target.MIDDLE_GOAL);
        }

        // TODO: Make pre-programmed out rotate and shot power shot
//        if (outwardPowerShotButton.isPressed()) {
//            robot.setTarget(Target.OUTWARD_POWER_SHOT);
//        }
//        if (middleGoalButton.isPressed()) {
//            robot.setTarget(Target.MIDDLE_POWER_SHOT);
//        }
//        if (inwardPowerShotButton.isPressed()) {
//            robot.setTarget(Target.INWARD_POWER_SHOT);
//        }
        if (powerShotButton.isPressed()) {
            robot.powerShot();
        }

        // Arm
        if (liftArmToggleButton.on()) {
            robot.arm.liftArm();
        } else if (liftArmToggleButton.off()) {
            robot.arm.lowerArm();
        }
        if (openClawToggleButton.on()) {
            robot.arm.openClaw();
        } else if (openClawToggleButton.off()) {
            robot.arm.closeClaw();
        }

        // Drive
        double radius = getLeftStickRadius();
        double angle = getLeftStickAngle();
        double rot = getRightX();
        robot.drive.teleopControl(radius, angle, rot, localControlToggleButton.on(), autoAimToggleButton.on());

        // Intake
        if (intakeOnToggleButton.on()) {
            robot.intake.turnOn();
        } else if (intakeOnToggleButton.off()) {
            robot.intake.turnOff();
        }

        // Shooter
        if (shootButton.isPressed()) {
            robot.shooter.shoot();
        }
    }

    public double getLeftStickRadius() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = Math.hypot(x,y);
        return r;
    }
    public double getLeftStickAngle() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double angle = Math.atan2(y,x);
        return angle;
    }
    public double getRightX() {
        double x = gamepad1.right_stick_x;
        return x;
    }
}
