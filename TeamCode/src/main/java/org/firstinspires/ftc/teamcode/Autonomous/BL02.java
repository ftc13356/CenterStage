package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class BL02 {
    PathChain yellow1, submersible;
    IDRobot robot;
    Pose starting = new Pose(7.5,103,0);
    Point score = new Point(14,119,Point.CARTESIAN);
    public BL02(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);
        robot.follower.setStartingPose(starting);

        yellow1 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(score, new Point(24,120,0)))
                .setLinearHeadingInterpolation(-Math.PI/4,0)
                .build();

        submersible = robot.follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(60,110,Point.CARTESIAN), new Point(60,96,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(-Math.PI/4, -Math.PI/2)
                .build();
    }
    /**
    * builds + follows path to go from current point -> score
     */
    public void placeSample () {
        robot.followPath(score, 0, -Math.PI/4,false,true);
        robot.queuer.addDelay(0.4);
        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, true);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.BUCKET, true);
        robot.queuer.addDelay(0.5);
        robot.setClaw(Claw.ClawStates.OPEN, false);
    }

    public void afterYellow(){
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.setArm(TelescopicArm.ArmStates.INTAKE, true);
        robot.setTwist(Twist.TwistStates.GRAB, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        placeSample();
    }

    public void placeSamples(){
        placeSample();
        //robot.followPath(yellow1, true);
        //afterYellow();
    }

    public void park() {
        robot.followPath(submersible,true);
    }

    public void update() {
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}