package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
public class BL02 {
    IDRobot robot;
    public static double x=24;
    public static double y=120;
    Point score = new Point(13.5,122,Point.CARTESIAN);
    public BL02(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);
        robot.follower.setStartingPose(new Pose(7.5,103,0));
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
        robot.sampleHigh(score);
        robot.followPath(new Point(x,y,Point.CARTESIAN), -PI/4, 0, false);
        afterYellow();
    }

    /**
    parks by submersible
     */
    public void park() {
        robot.followPath(new Point(60,110,Point.CARTESIAN), new Point(60,96, Point.CARTESIAN),Math.PI*3/4, -Math.PI/2, false);
    }

    public void update() {
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}