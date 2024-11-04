package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class SBAF {
    private Telemetry telemetryA;
    public static double distance = 24;
    private String navigation = "forward"; //1 = true
    private Follower follower;
    private Path forward, backward;
    private String forwardStatus, backwardStatus;

    private Pose endPose, midPose;

    public LinearOpMode op;

    public SBAF(LinearOpMode opmode){
        op = opmode;
        follower = new Follower(op.hardwareMap);
    }

    public void setBackdropGoalPose() {
            switch (navigation){
                default:
                case "forward":
                    endPose = new Pose(distance, 0, Point.CARTESIAN);
                    midPose = new Pose(-distance, 0, Point.CARTESIAN);
                    break;
                case "backward":
                    endPose = new Pose(-distance, 0, Point.CARTESIAN);
                    midPose = new Pose(distance, 0, Point.CARTESIAN);
                    break;
            }
        }

    public void buildPaths(){
        switch(navigation){
            default:
            case "forward":
                forward = new Path(new BezierLine(
                        new Point(0,0,Point.CARTESIAN),
                        new Point(endPose.getX(), endPose.getY(), Point.CARTESIAN)));
                forward.setConstantHeadingInterpolation(0);
                break;
            case "backward":
                backward = new Path(new BezierLine(
                        new Point(0,0, Point.CARTESIAN),
                        new Point(endPose.getX(), endPose.getY(), Point.CARTESIAN)));
                backward.setConstantHeadingInterpolation(0);
                break;
        }
    }

    public void update() {
        telemetryA.addData("direction", navigation);
        telemetryA.addData("status", forwardStatus);
        switch(navigation){
            default:
            case "forward":
                follower.followPath(forward);
                setNavigation("backward");
                setStatus(forwardStatus);
                break;
            case "backward":
                follower.followPath(backward);
                setNavigation("forward");
                setStatus(backwardStatus);
                break;
        }
    }

    public void setNavigation(String pathNavigation){
        navigation = pathNavigation;
        update();
    }

    public void setStatus(String pathStatus){
        switch(navigation){
            default:
            case "forward":
                forwardStatus = pathStatus;
                break;
            case "backward":
                backwardStatus = pathStatus;
        }
    }

}

