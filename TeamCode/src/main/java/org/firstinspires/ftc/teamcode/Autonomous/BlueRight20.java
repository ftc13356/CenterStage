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

    private Follower builder;

    public void runOpMode() throws InterruptedException{
        IDRobot robot = new IDRobot(this,false);
        robot.autoReset();
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        preload = builder.pathBuilder().addPath(
                new BezierLine(
                        new Point(10,63,Point.CARTESIAN),
                        new Point(35,63,Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(toRadians(0))
                .build();
        intake = builder.pathBuilder().addPath(
                        new BezierLine(
                                new Point(35,63,Point.CARTESIAN),
                                new Point(25,35,Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(toRadians(0))
                .build();
        deposit = builder.pathBuilder().addPath(
                        new BezierLine(
                                new Point(25,35,Point.CARTESIAN),
                                new Point(35,63,Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(toRadians(0))
                .build();
        park = builder.pathBuilder().addPath(
                        new BezierLine(
                                new Point(35,63,Point.CARTESIAN),
                                new Point(10,10,Point.CARTESIAN)
                        ))
                .setLinearHeadingInterpolation(toRadians(0), toRadians(90))
                .build();
        while(!isStopRequested() && opModeIsActive()){
            //preload
            robot.followPath(preload);
            robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN,true);
            robot.setTwist(Twist.TwistStates.PERPENDICULAR,true);
            robot.setFlip(Flip.FlipStates.SPECIMEN,true);
            //intake
            robot.setClaw(Claw.ClawStates.OPEN,false);
            robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB,true);
            robot.setTwist(Twist.TwistStates.GRAB,true);
            robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB,true);
            robot.followPath(intake);
            robot.setClaw(Claw.ClawStates.CLOSED,true);
            //deposit
            robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN,false);
            robot.setTwist(Twist.TwistStates.PERPENDICULAR,true);
            robot.setFlip(Flip.FlipStates.SPECIMEN,true);
            robot.followPath(deposit);
            robot.autoReset();
            //park
            robot.followPath(park);
        }
    }
}
