package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class LPark extends LinearOpMode{
    private PathChain park;

    public void runOpMode() throws InterruptedException{
        IDRobot robot = new IDRobot(this,false);
        robot.follower.setStartingPose(new Pose(7.5,103,0));
        park = robot.follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(7.5,103,Point.CARTESIAN),
                                new Point(50.6,119,Point.CARTESIAN),
                                new Point(65.1,99.5,Point.CARTESIAN)

                        ))
                .setTangentHeadingInterpolation()
                .build();
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            robot.followPath(park);
            robot.update();
        }
    }
}
