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
public class BlueRight20 extends LinearOpMode{
    private PathChain preload;
    private PathChain intake;
    private PathChain deposit;
    private PathChain park;

    public void runOpMode() throws InterruptedException{
        IDRobot robot = new IDRobot(this,false);
        robot.follower.setStartingPose(new Pose(10,57,0));
        robot.follower.setMaxPower(0.7);
        preload = robot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(10,57,Point.CARTESIAN),
                                new Point(35,65,Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(toRadians(0))
                .build();
        intake = robot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(35.000, 65.000, Point.CARTESIAN),
//                                new Point(23.000, 65.000, Point.CARTESIAN),
                                new Point(23, 44.000, Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(toRadians(0))
                .build();
        deposit = robot.follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Point(23.00, 44.000, Point.CARTESIAN),
                                new Point(23.00, 67.000, Point.CARTESIAN),
                                new Point(34.000, 67.000, Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(toRadians(0))
                .build();
        park = robot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(34,67,Point.CARTESIAN),
                                new Point(20,20,Point.CARTESIAN)
                        ))
                .setLinearHeadingInterpolation(toRadians(0), toRadians(90))
                .build();
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            //preload
            robot.followPath(preload);
            robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN,true);
            robot.setTwist(Twist.TwistStates.PARALLEL,true);
            robot.setFlip(Flip.FlipStates.SPECIMEN,true);
            robot.setClaw(Claw.ClawStates.OPEN,false);
            //intake
            robot.followPath(intake);
            robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB,true);
            robot.setTwist(Twist.TwistStates.SPECIMEN,true);
            robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB,true);
            robot.setClaw(Claw.ClawStates.CLOSED, false);
            //deposit
            robot.followPath(deposit);
            robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN,true);
            robot.setTwist(Twist.TwistStates.PARALLEL,true);
            robot.setFlip(Flip.FlipStates.SPECIMEN,true);
            robot.setClaw(Claw.ClawStates.OPEN,false);
            //park
            robot.followPath(park);
            robot.autoReset(true);
            robot.update();
            robot.queuer.setFirstLoop(false);
        }
    }
}