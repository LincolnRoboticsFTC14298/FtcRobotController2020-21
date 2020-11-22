package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Launcher 1 Motor", group="Prototype")
public class LauncherPrototype1Motor extends LinearOpMode {
    public final double MOTOR_STEP = 0.05; // Motor speed step

    @Override
    public void runOpMode() {
        double mSpeed = 0;

        DcMotor motor1 = hardwareMap.get(DcMotor.class, "motor1");

        // Uncomment one of these if you want to reverse one of the motors
        // May do this for the motors on opposite side
        // Forward may be clockwise but I forgot

        //motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setPower(0);

        waitForStart();

        while (opModeIsActive()) {
            // Up and down updates the first motor speed
            // Left and right updates the second motor speed
            if (gamepad1.dpad_up) {
                mSpeed += MOTOR_STEP;
            } if (gamepad1.dpad_down) {
                mSpeed -= MOTOR_STEP;
            }

            motor1.setPower(mSpeed);

            // Telemetry
            telemetry.addData("Motor power: ", mSpeed);
        }
    }
}