package org.firstinspires.ftc.teamcode.hardware.subsystems.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.Field.Alliance;
import org.firstinspires.ftc.teamcode.util.Field.Target;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.util.DriveConstants.encoderTicksToInches;

@Deprecated
public class Drive implements Subsystem {
    // Mecanum drive //

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet;

    //private static final FluentLogger logger = FluentLogger.forEnclosingClass();


    private DcMotorEx frontLeft, frontRight,  backLeft,backRight;
    public List<DcMotorEx> motors;

    private double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    private Target target = Target.HIGH_GOAL;
    private Alliance alliance = Alliance.BLUE;

    public boolean aligning = false;

    public Drive(HardwareMap hardwareMap) {
        // Initialize motors //
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRight = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Reverse direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void init() {
        packet = new TelemetryPacket();

        setMotorPowers(0,0,0,0);
        updateMotorPowers();

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void update() {
        packet = new TelemetryPacket();

        //logger.atFine().log("Motor powers: ", frontLeft.getPower(), frontRight.getPower(),backLeft.getPower(), backRight.getPower());
        //logger.atFiner().log("Diff of power - target: ",frontLeft.getPower() - frontLeftPower, frontRight.getPower() - frontRightPower,backLeft.getPower() - backLeftPower, backRight.getPower() - backRightPower);
        //logger.atFiner().log(target.toString(), alliance.toString());
        //logger.atFinest().log(frontLeft.getMode().toString());
        updateMotorPowers();

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        packet = new TelemetryPacket();

        setMotorPowers(0,0,0,0);
        updateMotorPowers();

        dashboard.sendTelemetryPacket(packet);
    }



    public void teleopControl(double radius, double angle, double rotation, double heading, boolean localControl) {
        angle -= Math.PI / 4; // Strafing angle
        if (localControl) {
            angle -= Math.PI / 2 + heading;
        }

        //logger.atFinest().log(String.valueOf(radius), String.valueOf(angle), String.valueOf(rotation));

        double fl = radius * Math.cos(angle) + rotation;
        double fr = radius * Math.sin(angle) - rotation;
        double bl = radius * Math.sin(angle) + rotation;
        double br = radius * Math.cos(angle) - rotation;

        setMotorPowers(fl, fr, bl, br);
    }

    public void targetLockControl(double radius, double angle, double rotation, double heading, boolean localControl) {
        angle -= Math.PI / 4; // Strafing angle
        if (localControl) {
            angle -= Math.PI / 2 + heading;
        }

        //logger.atFinest().log(String.valueOf(radius), String.valueOf(angle), String.valueOf(rotation));

        double fl = radius * Math.cos(angle) + rotation;
        double fr = radius * Math.sin(angle) - rotation;
        double bl = radius * Math.sin(angle) + rotation;
        double br = radius * Math.cos(angle) - rotation;

        setMotorPowers(fl, fr, bl, br);
    }


    public void setTarget(Target target) {
        this.target = target;
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Target getTarget() {
        return target;
    }
    public Alliance getAlliance() {
        return alliance;
    }

    // For RoadRunner
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, coefficients);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        // TODO: Clip powers to make sure it's between -1 and 1
        frontLeftPower = fl;
        frontRightPower = fr;
        backLeftPower = bl;
        backRightPower = br;
    }
    public void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void updateMotorPowers() {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        packet.addLine(String.format(Locale.ENGLISH, "Motor powers: %.6f fl   %.6f fr   %.6f bl   %.6f br",
                frontLeft.getPower(), frontRight.getPower(),
                backLeft.getPower(), backRight.getPower()));
    }
}
