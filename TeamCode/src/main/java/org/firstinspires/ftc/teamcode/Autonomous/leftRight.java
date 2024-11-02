package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "leftRight")
public class leftRight extends OpMode {
    private Telemetry telemetryA;
    public static double distance = 24;
    private boolean goingForward = true;
    private Follower follower;
    private PathChain firstLeft, firstRight;
    public String direction = "firstLeft";


    private Pose endPose;

    @Override
    public void init(){
        follower = new Follower(hardwareMap);

        switch (direction){
            default:
            case "left":
                endPose = new Pose(distance, distance, Point.CARTESIAN);
                break;
            case "right":
                endPose = new Pose(-distance, distance, Point.CARTESIAN);
                break;
        }

        firstLeft = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(0, distance, Point.CARTESIAN), new Point(endPose.getX(), endPose.getY(), Point.CARTESIAN)))
                .build();

        follower.followPath(firstLeft);

        firstRight = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(endPose.getX(), endPose.getY(), Point.CARTESIAN), new Point(0, distance, Point.CARTESIAN), new Point(0,0,Point.CARTESIAN)))
                .build();
    }

    public void loop() {
        follower.update();
        if(!follower.isBusy()){
            switch(direction) {
                default:
                case "firstLeft":
                    direction = "firstRight";
                    follower.followPath(firstRight);
                    break;
                case "firstRight":
                    direction = "firstLeft";
                    follower.followPath(firstLeft);
                }
            }
        telemetryA.addData("direction:", direction);
        follower.telemetryDebug(telemetryA);
        }
    }

