package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//@Disabled
@Autonomous(name = "OdomMoveTest")
public class RoadRunMoveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        logger2.log(RFLogger.Severity.CONFIG, RFLogger.Files.AUTONOMOUS_LOG, "Basic Robot Initialized");
        SampleMecanumDrive roadrun = new SampleMecanumDrive(this.hardwareMap);
        logger2.log(RFLogger.Severity.CONFIG, RFLogger.Files.AUTONOMOUS_LOG, "Mec drive initialized");
        Pose2d startPose = new Pose2d(35.25, 57.75, Math.toRadians(270));
        logger2.log(RFLogger.Severity.INFO, RFLogger.Files.AUTONOMOUS_LOG, "Starting Pose: " + startPose.getX() + startPose.getY() + startPose.getHeading());
        roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        logger2.log(RFLogger.Severity.CONFIG, RFLogger.Files.AUTONOMOUS_LOG, "DCMotor mode = run without encoder");

        roadrun.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;
        TrajectorySequence trajSeq2 = roadrun.trajectorySequenceBuilder(new Pose2d(35.25,57.75, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(35.25, 35.25,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(13.75, 35.25,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(13.75, 57.75,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(35.25, 57.75,Math.toRadians(270)))
                .build();

//        while (opModeIsActive()) {
            roadrun.followTrajectorySequence(trajSeq2);
            logger2.log(RFLogger.Severity.FINEST, RFLogger.Files.AUTONOMOUS_LOG, "Pass 1");
            roadrun.followTrajectorySequence(trajSeq2);
            logger2.log(RFLogger.Severity.FINEST, RFLogger.Files.AUTONOMOUS_LOG, "Pass 2");
            roadrun.followTrajectorySequence(trajSeq2);
            logger2.log(RFLogger.Severity.FINEST, RFLogger.Files.AUTONOMOUS_LOG, "Pass 3");
            roadrun.followTrajectorySequence(trajSeq2);
            logger2.log(RFLogger.Severity.FINEST, RFLogger.Files.AUTONOMOUS_LOG, "Pass 4");
            roadrun.followTrajectorySequence(trajSeq2);
            logger2.log(RFLogger.Severity.FINEST, RFLogger.Files.AUTONOMOUS_LOG, "Pass 5");
            roadrun.followTrajectorySequence(trajSeq2);
            logger2.log(RFLogger.Severity.FINEST, RFLogger.Files.AUTONOMOUS_LOG, "Pass 6");
            roadrun.followTrajectorySequence(trajSeq2);
            logger2.log(RFLogger.Severity.FINEST, RFLogger.Files.AUTONOMOUS_LOG, "Pass 7");
            roadrun.followTrajectorySequence(trajSeq2);
            logger2.log(RFLogger.Severity.FINEST, RFLogger.Files.AUTONOMOUS_LOG, "Pass 8");

//            roadrun.update();
//            telemetry.update();
//            robot.update();
//        }
    }
}
