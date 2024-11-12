package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Config
public class BLSS extends BasicRobot {
    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;
    private Follower follower;
    public LinearOpMode op;
    public BasicRobot basicBot;
    private PathChain specimeny, sampley;
    private Pose placeSpeci, placeSample, yellowSampleOne, yellowSampleTwo, yellowSampleThree;
    private boolean placedSpeci = false;

    public BLSS(LinearOpMode opmode){
        super(opmode,false);
        basicBot = new BasicRobot(op, false, false);
        op = opmode;
        follower = new Follower(op.hardwareMap);
        poseUpdater = new PoseUpdater(op.hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        placeSpeci = new Pose(43,113,Point.CARTESIAN);
        placeSample = new Pose(12,132,Point.CARTESIAN);

        yellowSampleOne = new Pose(43,113,Point.CARTESIAN);
        yellowSampleTwo = new Pose(44,123,Point.CARTESIAN);
        yellowSampleThree = new Pose(44,133,Point.CARTESIAN);

        specimeny = follower.pathBuilder()
                .addPath(new BezierLine(new Point(9.757, 84.983, Point.CARTESIAN), new Point(44, 76, Point.CARTESIAN)))
                .build();

        sampley = follower.pathBuilder() //set heading later!!!!!!!!!!!!!!
                .addPath(new BezierCurve(new Point(placeSpeci.getX(), placeSpeci.getY(), Point.CARTESIAN), new Point(29, 90, Point.CARTESIAN))) //Back up
                .addPath(new BezierCurve(new Point(29,90,Point.CARTESIAN), new Point(yellowSampleOne.getX(), yellowSampleOne.getY(),Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(yellowSampleOne.getX(), yellowSampleOne.getY(), Point.CARTESIAN), new Point(placeSample.getX(), placeSample.getY(), Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(placeSample.getX(), placeSample.getY(), Point.CARTESIAN), new Point(yellowSampleTwo.getX(), yellowSampleTwo.getY(),Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(yellowSampleTwo.getX(), yellowSampleTwo.getY(), Point.CARTESIAN), new Point(placeSample.getX(), placeSample.getY(), Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(placeSample.getX(), placeSample.getY(), Point.CARTESIAN), new Point(yellowSampleThree.getX(), yellowSampleThree.getY(), Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(yellowSampleThree.getX(), yellowSampleThree.getY(), Point.CARTESIAN), new Point(placeSample.getX(), placeSample.getY(), Point.CARTESIAN)))
                .build();
    }

    public void placeSpeci() {
        queuer.queue(false, true);
        followPathAsync(specimeny);
    }

    public void placeSample() {
        queuer.addDelay(5.0);
        followPathAsync(sampley);
    }

    public void update() {
        follower.update();
        queuer.setFirstLoop(false);

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