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

    private Pose endPose;

    public LinearOpMode op;

    public SBAF(LinearOpMode opmode){
        op = opmode;
        follower = new Follower(op.hardwareMap);
    }

    public void hmph () {
        forward = new Path(new BezierCurve(new Point(0,0,Point.CARTESIAN), new Point(0,distance, Point.CARTESIAN)));
        follower.followPath(forward);
    }
    /*


    public void setBackdropGoalPose() {
            switch (navigation){
                default:
                case "forward":
                    endPose = new Pose(distance, 0, Point.CARTESIAN);
                    break;
                case "backward":
                    endPose = new Pose(-distance, 0, Point.CARTESIAN);
                    break;
            }
        }

    public void buildPaths(){
        switch(navigation){
            default:
            case "forward":
                forward = new Path(new BezierLine(
                        new Point(0,0,Point.CARTESIAN),
                        new Point(endPose.getX(), endPose.getY(), Point.CARTESIAN))); //convert this to use buildPaths() and do 0,0, endPose, 0,0
                forward.setConstantHeadingInterpolation(0);
                break;
            case "backward":
                backward = new Path(new BezierLine(
                        new Point(0,0, Point.CARTESIAN),
                        new Point(endPose.getX(), endPose.getY(), Point.CARTESIAN)));//convert this to use buildPaths() and do 0,0, endPose, 0,0
                backward.setConstantHeadingInterpolation(0);
                break;
        }
    }

    public void update() {
        telemetryA.addData("direction", navigation);
        telemetryA.addData("running", forwardStatus);
        switch(navigation){
            default:
            case "forward":
                follower.followPath(forward);
                setNavigation("backward");
                forwardStatus = follower.atParametricEnd();
                setStatus(forwardStatus);
                break;
            case "backward":
                follower.followPath(backward);
                setNavigation("forward");
                backwardStatus = follower.atParametricEnd();
                setStatus(backwardStatus);
                break;
        }
    }

    public void setNavigation(String pathNavigation){
        navigation = pathNavigation;
        update();
    }

    public void setStatus(Boolean pathStatus){
        switch(navigation){
            default:
            case "forward":
                forwardStatus = pathStatus;
                break;
            case "backward":
                backwardStatus = pathStatus;
        }
    }*/

}