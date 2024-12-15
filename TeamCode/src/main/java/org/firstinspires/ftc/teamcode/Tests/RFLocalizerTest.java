package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/**
 * Warren ZHou
 * 8/22
 * TeleOp + RR odometry test
 */
@Disabled
@Config
@TeleOp(name = "OdometryLocalizerTest")
public class RFLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this,false);
        SampleMecanumDrive roadrun = new SampleMecanumDrive(this.hardwareMap, Tracker.TrackType.ROADRUN_ODOMETRY);
        roadrun.setPoseEstimate(new Pose2d(0,0,toRadians(-90)));
        waitForStart();
        while (opModeIsActive()) {
            robot.update();
            roadrun.update();
            roadrun.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            packet.put("POSE",roadrun.getPoseEstimate());
//            packet.put("rrPose", drive.getPoseEstimate());
            packet.put("rrPOVVelocity", roadrun.getPoseVelocity());
            packet.put("veloMag", roadrun.getPoseVelocity().vec().norm());
        }
    }
}