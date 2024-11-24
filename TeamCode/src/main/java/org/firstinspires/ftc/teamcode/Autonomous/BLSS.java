package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Config
public class BLSS extends BasicRobot {
    DashboardPoseTracker dashboardPoseTracker;
    PoseUpdater poseUpdater;
    Follower follower;
    PathChain specimeny, sampley;
    Pose speciPose, samplePose, yellowSampleOne, yellowSampleTwo, yellowSampleThree;

    public BLSS(LinearOpMode opmode){
        super(opmode,false);
        follower = new Follower(opmode.hardwareMap);
        poseUpdater = new PoseUpdater(opmode.hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        speciPose = new Pose(33.243,28.017,Point.CARTESIAN);
        samplePose = new Pose(10,36.017,Point.CARTESIAN);

        yellowSampleOne = new Pose(26.243,24.017,Point.CARTESIAN);
        yellowSampleTwo = new Pose(26.243,30.017,Point.CARTESIAN);
        yellowSampleThree = new Pose(26.243,36.017,Point.CARTESIAN);

        specimeny = follower.pathBuilder()
                .addPath(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(34.243, -8.983, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(0,0)
                //.addPath(new BezierLine(new Point(9.757, 84.983, Point.CARTESIAN), new Point(44, 76, Point.CARTESIAN)))
                .build();
        sampley = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(speciPose.getX(), speciPose.getY(), Point.CARTESIAN), new Point(26.243, 3.017, Point.CARTESIAN)))
                .setPathEndTimeoutConstraint(2.0)
                .addPath(new BezierCurve(new Point(19.243,5.017,Point.CARTESIAN), new Point(yellowSampleOne.getX(), yellowSampleOne.getY(),Point.CARTESIAN)))
                .setPathEndTimeoutConstraint(2.0)
                .addPath(new BezierCurve(new Point(yellowSampleOne.getX(), yellowSampleOne.getY(), Point.CARTESIAN), new Point(samplePose.getX(), samplePose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI/2,Math.PI/2)
                .addPath(new BezierCurve(new Point(samplePose.getX(), samplePose.getY(), Point.CARTESIAN), new Point(yellowSampleTwo.getX(), yellowSampleTwo.getY(),Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI/2,Math.PI*3/4)
                .addPath(new BezierCurve(new Point(yellowSampleTwo.getX(), yellowSampleTwo.getY(), Point.CARTESIAN), new Point(samplePose.getX(), samplePose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4,Math.PI/2)
                .addPath(new BezierCurve(new Point(samplePose.getX(), samplePose.getY(), Point.CARTESIAN), new Point(yellowSampleThree.getX(), yellowSampleThree.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI/2,Math.PI*3/4)
                .addPath(new BezierCurve(new Point(yellowSampleThree.getX(), yellowSampleThree.getY(), Point.CARTESIAN), new Point(samplePose.getX(), samplePose.getY(), Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI/2,Math.PI/2)
                .build();

        follower.setMaxPower(0.7);
    }

    public void placeSpeci() {
        queuer.queue(false, true);
        followPathAsync(specimeny);
    }

    public void placeSample() {
        queuer.addDelay(1.5);
        followPathAsync(sampley);
    }

    public void updateFollower() {
        follower.update();
        queuer.setFirstLoop(false);

        poseUpdater.update();
        packet.put("x", poseUpdater.getPose().getX());
        packet.put("y", poseUpdater.getPose().getY());
        packet.put("heading", poseUpdater.getPose().getHeading());
        packet.put("total heading", poseUpdater.getTotalHeading());
        update();
    }

    public void followPathAsync(PathChain traj){
        if(queuer.queue(false, !queuer.isFirstLoop()&&queuer.isExecuted()&&!follower.isBusy())){
            if(!follower.isBusy()) {
                follower.followPath(traj);
            }
        }
    }
}