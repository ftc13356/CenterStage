package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class BL20 {
    PoseUpdater poseUpdater;
    Follower builder;
    PathChain scoreSample, yellow1, yellow2, submersible;
    IDRobot robot;

    Pose starting = new Pose(12,108,0);

    public BL20(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);

        builder = new Follower(opmode.hardwareMap);
        builder.setStartingPose(starting);

        yellow1 = builder.pathBuilder()
                .addPath(new BezierCurve(new Point(12,108,Point.CARTESIAN), new Point(24,120,0)))
                .setLinearHeadingInterpolation(0,0)
                .build();

        yellow2 = builder.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(24,130,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4, 0)
                .build();

        submersible = builder.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(60,110,Point.CARTESIAN), new Point(60,96,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4, -Math.PI/2)
                .build();

        robot.autoReset();
        robot.setClaw(Claw.ClawStates.CLOSED, false);
    }
    /**
    * builds + follows path to go from current point -> (20,124)
     */
    public void placeSample () {
        Pose current = builder.getPose();
        scoreSample = builder.pathBuilder()
                .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), new Point(20,124,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(0, Math.PI*3/4)
                .build();
        robot.followPath(scoreSample, true);
    }

    public void afterYellow(){
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.setArm(TelescopicArm.ArmStates.INTAKE, true);
        robot.setTwist(Twist.TwistStates.GRAB, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.addDelay(3.0);
        robot.setClaw(Claw.ClawStates.CLOSED, false);

        placeSample();
        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.addDelay(3.0);
        robot.setClaw(Claw.ClawStates.OPEN, false);
    }

    public void placeSamples(){
        robot.followPath(yellow1, true);
        afterYellow();
        robot.followPath(yellow2,true);
        afterYellow();
    }

    public void park() {
        robot.followPath(submersible,true);
    }

    public void updateFollower() {
        builder.update();
        robot.queuer.setFirstLoop(false);

        poseUpdater.update();
        robot.packet.put("x", poseUpdater.getPose().getX());
        robot.packet.put("y", poseUpdater.getPose().getY());
        robot.packet.put("heading", poseUpdater.getPose().getHeading());
        robot.packet.put("total heading", poseUpdater.getTotalHeading());
        robot.update();
    }
}