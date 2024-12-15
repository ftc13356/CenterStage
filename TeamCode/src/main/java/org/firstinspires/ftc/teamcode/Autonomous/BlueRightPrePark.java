package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Config
@Autonomous (name = "BlueRightPrePark")
public class BlueRightPrePark extends LinearOpMode{
    private PathChain preload;
    private PathChain park;

    public void runOpMode() throws InterruptedException{
        IDRobot robot = new IDRobot(this,false);
        robot.follower.setStartingPose(new Pose(10,57,0));
        robot.follower.setMaxPower(0.7);
        preload = robot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(10,57,Point.CARTESIAN),
                                new Point(34,63,Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(toRadians(0))
                .build();
        park = robot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(34,63,Point.CARTESIAN),
                                new Point(20,20,Point.CARTESIAN)
                        ))
                .setLinearHeadingInterpolation(toRadians(0), toRadians(90))
                .build();
        waitForStart();
        robot.queuer.reset();
        while(!isStopRequested() && opModeIsActive()){
            //preload
            robot.followPath(preload);
            robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN,true);
            robot.setTwist(Twist.TwistStates.PARALLEL,true);
            robot.setFlip(Flip.FlipStates.SPECIMEN,true);
            robot.setClaw(Claw.ClawStates.OPEN,false);
            robot.followPath(park);
            robot.autoReset(true);
            robot.update();
            robot.queuer.setFirstLoop(false);
        }
    }
}
