package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.drive.DriveConstants.MOTOR_VELO_PID;

@TeleOp(name="Arm PID Tuner", group="Tuner")
public class ArmPIDTuner extends OpMode {
    private SubsystemManager subsystemManager = new SubsystemManager();
    private Arm arm;
    private RadicalGamepad gamepad;

    private double lastKp, lastKi, lastKd, lastKf;

    private ElapsedTime elapsedTime;
    private double loopTime = 5;
    private boolean nextLift = true;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
        subsystemManager.add(arm);
        gamepad = new RadicalGamepad(gamepad1);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        lastKp = arm.POS_PIDF.p;
        lastKi = arm.POS_PIDF.i;
        lastKd = arm.POS_PIDF.d;
        lastKf = arm.POS_PIDF.f;

        elapsedTime = new ElapsedTime();
    }

    @Override
    public void start() {
        subsystemManager.start();
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        gamepad.update();

        if (gamepad.a) {
            arm.openClaw();
        } else if (gamepad.b) {
            arm.closeClaw();
        }

        // Arm goes back and forth
        if (nextLift && elapsedTime.seconds() > loopTime) {
            elapsedTime.reset();
            arm.setArmAngle(Math.toRadians(100));
            nextLift = false;
        } else if (elapsedTime.seconds() > loopTime) {
            elapsedTime.reset();
            arm.setArmAngle(Math.toRadians(30));
            nextLift = true;
        }

        updatePIDF();
        telemetry.addData("PIDF coeff", arm.POS_PIDF.toString());
        telemetry.addData("Target", arm.getTargetArmAngle());
        telemetry.addData("Current", arm.getArmAngle());
        telemetry.update();
        subsystemManager.update();
    }

    public void updatePIDF() {
        if (lastKp != MOTOR_VELO_PID.p || lastKd != MOTOR_VELO_PID.d
                    || lastKi != MOTOR_VELO_PID.i || lastKf != MOTOR_VELO_PID.f) {
            arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

            lastKp = MOTOR_VELO_PID.p;
            lastKi = MOTOR_VELO_PID.i;
            lastKd = MOTOR_VELO_PID.d;
            lastKf = MOTOR_VELO_PID.f;
        }
    }
}
