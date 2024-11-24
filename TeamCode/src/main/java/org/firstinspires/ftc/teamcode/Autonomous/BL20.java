package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class BL20 {
    PoseUpdater poseUpdater;
    Follower follower;
    PathChain scoreSampley, getSampley1, getSampley2, submersibley;
    IDRobot robot;

    Pose starting = new Pose(12,108,0);

    public BL20(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);

        follower = new Follower(opmode.hardwareMap);
        follower.setStartingPose(starting);

        getSampley1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(12,108,Point.CARTESIAN), new Point(24,120,0)))
                .setLinearHeadingInterpolation(0,0)
                .build();

        getSampley2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(24,130,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4, 0)
                .build();

        submersibley = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(60,110,Point.CARTESIAN), new Point(60,96,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4, -Math.PI/2)
                .build();

        robot.autoReset();
        robot.setClaw(Claw.ClawStates.CLOSED, false);
    }
    /**
    * builds + follows path to go from current point -> sample-scoring
     */
    public void placeSample () {
        Pose current = follower.getPose();
        scoreSampley = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), new Point(20,124,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(0, Math.PI*3/4)
                .build();
        robot.followPath(scoreSampley, true);
    }

    /**
     * follows paths + sets claw and arm to pick up and places yellow samples 1,2
     */

    public void afterGetSampley(){
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.setArm(TelescopicArm.ArmStates.INTAKE, true);
        robot.queuer.addDelay(3.0);

        placeSample();
        robot.setClaw(Claw.ClawStates.CLOSED, true);
        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, true);
        robot.queuer.addDelay(3.0);
    }

    public void placeSamples(){
        robot.followPath(getSampley1, true);
        afterGetSampley();
        robot.followPath(getSampley2,true);
        afterGetSampley();
    }

    /**
     * parks by submersible
     */
    public void park() {
        robot.followPath(submersibley,true);
    }

    public void updateFollower() {
        follower.update();
        robot.queuer.setFirstLoop(false);

        poseUpdater.update();
        robot.packet.put("x", poseUpdater.getPose().getX());
        robot.packet.put("y", poseUpdater.getPose().getY());
        robot.packet.put("heading", poseUpdater.getPose().getHeading());
        robot.packet.put("total heading", poseUpdater.getTotalHeading());
        robot.update();
    }
}