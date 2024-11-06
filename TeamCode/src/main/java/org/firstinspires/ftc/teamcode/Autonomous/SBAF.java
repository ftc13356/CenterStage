package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class SBAF {
    private Telemetry telemetryA;

    public static double distance = 24;
    private String navigation = "forward";
    private Follower follower;
    private Path forward, backward;
    private Boolean forwardStatus, backwardStatus;

    private Pose forwardEndPose, backwardEndPose;

    public LinearOpMode op;

    public SBAF(LinearOpMode opmode){
        op = opmode;
        follower = new Follower(op.hardwareMap);
    }


    public void hmph () {
        forward = new Path(new BezierCurve(
                new Point(0,0,Point.CARTESIAN),
                new Point(distance,0, Point.CARTESIAN)));
        follower.followPath(forward);
    }

    public void help () {
        if(!follower.atParametricEnd()){
            follower.update();
        }
        telemetryA.addData("what", navigation);
    }


    public void setBackdropGoalPose() {
        forwardEndPose = new Pose(distance, 0, Point.CARTESIAN);

        backwardEndPose = new Pose(-distance, 0, Point.CARTESIAN);


        }

    public void buildPaths(){
                forward = new Path(new BezierLine(
                        new Point(0,0,Point.CARTESIAN),
                        new Point(forwardEndPose.getX(), forwardEndPose.getY(), Point.CARTESIAN))); //convert this to use buildPaths() and do 0,0, endPose, 0,0
                forward.setConstantHeadingInterpolation(0);

                backward = new Path(new BezierLine(
                        new Point(0,0, Point.CARTESIAN),
                        new Point(backwardEndPose.getX(), backwardEndPose.getY(), Point.CARTESIAN)));//convert this to use buildPaths() and do 0,0, endPose, 0,0
                backward.setConstantHeadingInterpolation(0);


    }

    public void updateFollower() {
        telemetryA.addData("direction", navigation);
        follower.update(); //Updates path
        switch(navigation){
            default:
            case "forward":
                telemetryA.addData("not running", forwardStatus);
                follower.followPath(forward);
                forwardStatus = follower.atParametricEnd();
                setStatus(navigation, forwardStatus);
                if(forwardStatus){
                    setNavigation("backward");
                }
                break;
            case "backward":
                telemetryA.addData("not running", backwardStatus);
                follower.followPath(backward);
                backwardStatus = follower.atParametricEnd();
                setStatus(navigation, backwardStatus);
                if(backwardStatus){
                    setNavigation("forward");
                }
                break;
        }
    }

    public void setNavigation(String pathNavigation){
        navigation = pathNavigation;
        updateFollower();
    }

    public void setStatus(String Navigation, Boolean pathStatus){
        switch(Navigation){
            default:
            case "forward":
                forwardStatus = pathStatus;
                break;
            case "backward":
                backwardStatus = pathStatus;
        }
    }
}