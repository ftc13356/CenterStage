package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/**
 * Warren ZHou
 * 8/22
 * TeleOp + RR odometry test
 */
//@Disabled
//@Disabled
//@Disabled
@Config
@Autonomous(name = "OdometryLocalizerTest")
public class RFLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this,false);
//        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap, Tracker.TrackType.ROADRUN_ODOMETRY);
//        boolean imud = false, doneImud = false;
        robot.roadrun.setPoseEstimate(new Pose2d(0,0,toRadians(-90)));

        waitForStart();
        robot.roadrun.setPoseEstimate(new Pose2d(0,0,toRadians(-90)));

        while (opModeIsActive()) {
            robot.update();
//            drive.update();
//            if(!imud && BasicRobot.time>4){
//                drive.startIMU();
//                imud=true;
//            }
//            if(!doneImud && BasicRobot.time>5){
//                drive.changeIMUInterval();
//                doneImud=true;
//            }
            robot.roadrun.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
//            packet.put("rrPose", drive.getPoseEstimate());
            packet.put("rrPOVVelocity", robot.roadrun.getPoseVelocity());
        }
    }
}