package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RoadRunMoveTest")
public class RoadRunMoveTest extends LinearOpMode {
    public static double DISTANCE = 48; // in

    @Override
    public void runOpMode() throws InterruptedException {
        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.ODOMETRY, false, false, 0);

        Pose2d startPose = new Pose2d(55.5, -53.5, 0);

        robot.roadrun.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence trajSeq = robot.roadrun.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(55.5, -29.5))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(7.5, -29.5))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(7.5, -53.5))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(55.5, -53.5))
                .build();
        while (opModeIsActive()) {
            robot.followTrajectorySequenceAsync(trajSeq);
            robot.followTrajectorySequenceAsync(trajSeq);
            robot.setFirstLoop(false);
            robot.roadrun.update();
        }
    }
}