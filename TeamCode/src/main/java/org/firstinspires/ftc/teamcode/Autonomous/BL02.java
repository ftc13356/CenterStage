package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class BL02 {
    PathChain yellow1, submersible;
    IDRobot robot;
    Pose starting = new Pose(7.5,103,0);
    public static double x=24;
    public static double y=120;
    Point score = new Point(13.5,122,Point.CARTESIAN);
    public BL02(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);
        robot.follower.setStartingPose(starting);

        yellow1 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(score, new Point(x,y,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(-PI/4,0)
                .build();

        submersible = robot.follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(60,110,Point.CARTESIAN), new Point(60,96,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(-PI/4, -PI/2)
                .build();
    }
    /**
    * builds + follows path to go from current point -> score
     */
    public void placeSample () {
        robot.followPath(score, 0, -PI/4,false,true);
        robot.queuer.addDelay(1);
        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.BUCKET, true);
        robot.queuer.addDelay(0.5);
        robot.setClaw(Claw.ClawStates.OPEN, false);
    }

    public void afterYellow(){
        double height=4, length=15;
        double ext = length-7, rot = 180/PI *Math.atan2(height, length);
        robot.queuer.addDelay(1.0);
        robot.setArm(ext, rot, true);
        robot.queuer.addDelay(1.0);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
//        robot.setClaw(Claw.ClawStates.CLOSED, false);
//        placeSample();
    }

    public void placeSamples(){
        placeSample();
        robot.followPath(yellow1, false);
        afterYellow();
    }

    public void park() {
        robot.followPath(submersible,true);
    }

    public void update() {
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}