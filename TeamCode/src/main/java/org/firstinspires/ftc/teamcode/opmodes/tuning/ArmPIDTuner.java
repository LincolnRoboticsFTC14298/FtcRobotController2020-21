package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.hardware.SubsystemManager;
import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

@TeleOp(name="Arm PID Tuner", group="Tuner")
@Config
public class ArmPIDTuner extends OpMode {
    private SubsystemManager subsystemManager = new SubsystemManager();
    private Arm arm;
    private RadicalGamepad gamepad;

    private double lastKp, lastKi, lastKd, lastKf;

    private ElapsedTime elapsedTime;
    private double loopTime = 5;
    private boolean nextLift = true;

    public static PIDFCoefficients pidfCoefficients = Arm.PIDF_WITHOUT_WOBBLE;

    private Telemetry telemetry1;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
        subsystemManager.add(arm);
        gamepad = new RadicalGamepad(gamepad1);

        //arm.init();
        arm.resetDefaultAngle();

        telemetry1 = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        lastKp = pidfCoefficients.p;
        lastKi = pidfCoefficients.i;
        lastKd = pidfCoefficients.d;
        lastKf = pidfCoefficients.f;

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
            arm.setArmAngle(Math.toRadians(90));
            nextLift = false;
        } else if (elapsedTime.seconds() > loopTime) {
            elapsedTime.reset();
            arm.middle();
            nextLift = true;
        }

        updatePIDF();
        arm.update();
        arm.updateLogging();
        arm.updateMotorAndServoValues();
        telemetry1.addData("P", lastKp);
        telemetry1.addData("PIDF coeff", pidfCoefficients.toString());
        telemetry1.addData("Target", arm.getTargetArmAngle());
        telemetry1.addData("Current", arm.getArmAngle());
        telemetry1.update();
        //subsystemManager.update();
    }

    public void updatePIDF() {
        if (lastKp != pidfCoefficients.p || lastKd != pidfCoefficients.d
                    || lastKi != pidfCoefficients.i || lastKf != pidfCoefficients.f) {
            arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            lastKp = pidfCoefficients.p;
            lastKi = pidfCoefficients.i;
            lastKd = pidfCoefficients.d;
            lastKf = pidfCoefficients.f;
        }
    }
}
