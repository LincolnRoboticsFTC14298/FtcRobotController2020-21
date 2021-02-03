package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotlib.hardware.gamepad.RadicalGamepad;
import org.firstinspires.ftc.teamcode.hardware.subsystems.dream.Elevator;

@Config
@TeleOp(name="Elevator Tuner", group="Tuner")
@Disabled
public class ElevatorTuner extends OpMode {
    private Elevator elevator;
    private RadicalGamepad gamepad;

    @Override
    public void init() {
        elevator = new Elevator(hardwareMap);
        gamepad = new RadicalGamepad(gamepad1);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        gamepad.update();

        if (gamepad.a) {
            elevator.raise();
        } else if (gamepad.b) {
            elevator.lower();
        }

        elevator.update();

        telemetry.addData("Is Up: ", elevator.isUp());
        telemetry.addData("Is Down: ", elevator.isDown());
        telemetry.update();
    }
}
