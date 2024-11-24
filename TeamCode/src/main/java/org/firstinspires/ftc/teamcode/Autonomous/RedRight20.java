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
public class RedRight20 extends LinearOpMode{
    private PathChain preload;
    private PathChain spec1;
    private PathChain spec2;
    private PathChain deposit1;
    private PathChain intake2;
    private PathChain deposit2;
    private PathChain park;

    private Follower builder;

    public void runOpMode() throws InterruptedException{
        IDRobot robot = new IDRobot(this,false);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
        robot.setArm(TelescopicArm.ArmStates.HOVER, true);
        robot.setFlip(Flip.FlipStates.RESET, true);
        robot.setTwist(Twist.TwistStates.GRAB, true);
        preload = builder.pathBuilder().addPath(
                new BezierLine(
                        new Point(134,87,Point.CARTESIAN),
                        new Point(106, 75, Point.CARTESIAN)
                )).setConstantHeadingInterpolation(toRadians(0)).build();
        spec1 = builder.pathBuilder().addPath(
                new BezierLine(
                        new Point(106, 75, Point.CARTESIAN),
                        new Point(116,122, Point.CARTESIAN)
                )).setConstantHeadingInterpolation(toRadians(0)).build();
        spec2 = builder.pathBuilder().addPath(
                new BezierLine(
                        new Point(116, 122, Point.CARTESIAN),
                        new Point(116,131, Point.CARTESIAN)
                )).setConstantHeadingInterpolation(toRadians(0)).build();
        deposit1 = builder.pathBuilder().addPath(
                new BezierLine(
                        new Point(116, 131, Point.CARTESIAN),
                        new Point(108,75, Point.CARTESIAN)
                )).setLinearHeadingInterpolation(toRadians(0), toRadians(0)).build();
        intake2 = builder.pathBuilder().addPath(
                new BezierLine(
                        new Point(108,75,Point.CARTESIAN),
                        new Point(120,96,Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(toRadians(0),toRadians(45))
                .build();
        deposit2 = builder.pathBuilder().addPath(
                new BezierLine(
                        new Point(120,96,Point.CARTESIAN),
                        new Point(108,74,Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(toRadians(45),toRadians(0))
                .build();
        park = builder.pathBuilder().addPath(
                new BezierLine(
                        new Point(108,74,Point.CARTESIAN),
                        new Point(130,133,Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(toRadians(0),toRadians(90))
                .build();
        while(!isStopRequested() && opModeIsActive()){
            //preload
            robot.followPath(preload);
            robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, true);
            robot.setFlip(Flip.FlipStates.SPECIMEN, true);
            //reset to grab
            robot.setClaw(Claw.ClawStates.OPEN, false);
            robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, false);
            robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, false);
            robot.setTwist(Twist.TwistStates.GRAB, false);
            //flip1
            robot.followPath(spec1);
            robot.setClaw(Claw.ClawStates.CLOSED, false);
            robot.setArm(TelescopicArm.ArmStates.SAMPLE_DROP, false);
            robot.setClaw(Claw.ClawStates.OPEN, false);
            //flip2
            robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, false);
            robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, false);
            robot.setTwist(Twist.TwistStates.GRAB, false);
            robot.followPath(spec2);
            robot.setClaw(Claw.ClawStates.CLOSED, false);
            robot.setArm(TelescopicArm.ArmStates.SAMPLE_DROP, false);
            robot.setClaw(Claw.ClawStates.OPEN, false);
            //intake1
            robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, false);
            robot.setTwist(Twist.TwistStates.GRAB, false);
            robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, false);
            robot.setClaw(Claw.ClawStates.CLOSED,false);
            //deposit1
            robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, true);
            robot.setFlip(Flip.FlipStates.SPECIMEN, true);
            robot.followPath(deposit1);
            robot.setClaw(Claw.ClawStates.OPEN, false);
            robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, false);
            robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, false);
            robot.setTwist(Twist.TwistStates.GRAB, false);
            //intake2
            robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, false);
            robot.setTwist(Twist.TwistStates.GRAB, false);
            robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, false);
            robot.setClaw(Claw.ClawStates.CLOSED,false);
            robot.followPath(intake2);
            robot.setClaw(Claw.ClawStates.CLOSED, false);
            //deposit2
            robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, true);
            robot.setFlip(Flip.FlipStates.SPECIMEN, true);
            robot.followPath(deposit2);
            robot.autoReset();
            robot.followPath(park);
        }
    }
}
