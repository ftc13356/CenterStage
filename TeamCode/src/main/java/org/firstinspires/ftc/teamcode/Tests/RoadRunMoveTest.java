package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

//@Disabled

/**
 * Warren Zhou
 * 9/6/23
 * Odom localizer 8 24x24 squares
 */
//@Disabled
@Disabled
@Autonomous(name = "OdomMoveTest")
public class RoadRunMoveTest extends LinearOpMode {
    Queuer queuer;
    SampleMecanumDrive roadrun;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        roadrun = new SampleMecanumDrive(this.hardwareMap, Tracker.TrackType.ROADRUN_ODOMETRY);
        Pose2d startPose = new Pose2d(35.25, 57.75, Math.toRadians(270));
        roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roadrun.setPoseEstimate(startPose);
        roadrun.setPoseEstimate(startPose);
        queuer = new Queuer();
        int loops = 0;
        roadrun.update();
        robot.update();
        waitForStart();
        if (isStopRequested()) return;
        TrajectorySequence trajSeq2 = roadrun.trajectorySequenceBuilder(new Pose2d(35.25,57.75, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(35.25, 35.25,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(13.75, 35.25,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(13.75, 57.75,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(35.25, 57.75,Math.toRadians(270)))
                .build();

        //        while (opModeIsActive()) {
        resetRuntime();
        BasicRobot.time = 0;
        while(!isStopRequested()&&opModeIsActive()) {
            for(int i=0;i<8;i++){
                followTrajAsync(trajSeq2);
            }
            loops++;
            packet.put("loopTime", loops/BasicRobot.time);
            packet.put("errah", currentPose.minus(startPose));
            queuer.setFirstLoop(false);
            robot.update();
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
