package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.hardware.Robot
import org.firstinspires.ftc.teamcode.util.Field
import org.firstinspires.ftc.teamcode.util.Field.conditionalMirror

@Config
open class WobbleGoal(var alliance: Field.Alliance) : OpMode() {
    lateinit var robot: Robot
    @JvmField var startPose: Pose2d = Pose2d(0.0, 0.0, 0.0);

    lateinit var traj1: Trajectory
    lateinit var traj2: Trajectory
    lateinit var traj3: Trajectory

    override fun init() {
        robot = Robot(this)

        robot.setPoseEstimate(conditionalMirror(startPose, alliance))

        traj1 = robot.drive.trajectoryBuilder(robot.localizer.poseEstimate)
                .back(1.0)
                .build()
        traj2 = robot.drive.trajectoryBuilder(traj1.end())
                .back(1.0)
                .build()
        traj3 = robot.drive.trajectoryBuilder(traj3.end())
                .back(1.0)
                .build()
    }

    override fun start() {
        robot.arm.lift()

        robot.drive.followTrajectory(traj1);

        robot.arm.lower()
        // Sleep?
        robot.arm.openClaw()

        // Log wobble goal position

        robot.drive.followTrajectory(traj2)

        robot.arm.closeClaw()
        // Sleep?
        robot.arm.lift()

        robot.drive.followTrajectory(traj3)

        robot.arm.lower()
        // Sleep?
        robot.arm.openClaw()
    }

    override fun loop() {

    }
}

@Autonomous(name = "Wobble Goal Blue", group = "Autonomous")
@Disabled
class WobbleGoalBlue : WobbleGoal(Field.Alliance.BLUE)

@Autonomous(name = "Wobble Goal Red", group = "Autonomous")
@Disabled
class WobbleGoalRed : WobbleGoal(Field.Alliance.RED)