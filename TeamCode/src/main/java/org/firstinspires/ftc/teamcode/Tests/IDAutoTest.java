package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class IDAutoTest extends LinearOpMode {
    Queuer queuer;
    SampleMecanumDrive roadrun;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        roadrun = new SampleMecanumDrive(this.hardwareMap, Tracker.TrackType.ROADRUN_ODOMETRY);
        Pose2d startPose = new Pose2d(-32,-59, toRadians(-90));
        roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roadrun.setPoseEstimate(startPose);
        queuer = new Queuer();
        int loops = 0;
        TrajectorySequence trajSeq1 = roadrun.trajectorySequenceBuilder(new Pose2d(-32,-59, toRadians(-90)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-15,-35.5),toRadians(90))
                .waitSeconds(1)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-45,-45.5, toRadians(225)))
                .waitSeconds(0.1)
                .build();
        TrajectorySequence trajSeq2 = roadrun.trajectorySequenceBuilder(new Pose2d(-45,-45.5, toRadians(225)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-35,-15.5, toRadians(-170)),toRadians(70))
                .waitSeconds(0.25)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-45,-45.5, toRadians(225)),toRadians(250))
                .waitSeconds(0.1)
                .build();
        TrajectorySequence trajSeq3 = roadrun.trajectorySequenceBuilder(new Pose2d(-45,-45.5, toRadians(225)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-32,-52.5, toRadians(-90)))
                .build();
        waitForStart();
        if (isStopRequested()) return;


        //        while (opModeIsActive()) {
        resetRuntime();
        BasicRobot.time = 0;
        while(!isStopRequested()&&opModeIsActive()) {
            followTrajAsync(trajSeq1);
            for(int a=0; a<7; a++) {
                followTrajAsync(trajSeq2);
            }
            followTrajAsync(trajSeq3);
            loops++;
            robot.update();
            queuer.setFirstLoop(false);
            packet.put("errah", new Pose2d(-32,-52.5, toRadians(-90)).minus(currentPose).toString());
            roadrun.update();
        }
    }
    public void followTrajAsync(TrajectorySequence traj){
        if(queuer.queue(false, !roadrun.isBusy())){
            if(!roadrun.isBusy()) {
                roadrun.followTrajectorySequenceAsync(traj);
            }
        }
    }

}

