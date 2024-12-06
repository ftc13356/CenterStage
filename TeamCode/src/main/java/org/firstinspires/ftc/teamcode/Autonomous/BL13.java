//ignore this + BlueLeft31

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class BL13 {
    IDRobot robot;

    public BL13(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);
        robot.follower.setStartingPose(new Pose(7.5,103,0));
    }
    /**
     * builds + follows path to go from current point -> sample-scoring
     */
    public void placeSample () {
        Point score = new Point(20, 124, Point.CARTESIAN);
        robot.followPath(score, 0, Math.PI/4,true);

        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, true);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.BUCKET, true);
        robot.setClaw(Claw.ClawStates.OPEN, false);
    }

    /**
     * follows path from starting point -> specimen-scoring
     */
    public void placeSpeci(){
        robot.queuer.queue(false, true);
        robot.followPath(new Point(36,72,Point.CARTESIAN), 0,0,false);
        robot.queuer.addDelay(3.0);
    }
    /**
     * follows paths + sets claw and arm to pick up and places yellow samples 1,2,3
     */
    public void afterYellow(){ //copy paste from bl02
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.setArm(TelescopicArm.ArmStates.INTAKE, true);
        robot.queuer.addDelay(3.0);

        placeSample();
        robot.setClaw(Claw.ClawStates.CLOSED, true);
        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, true);
        robot.queuer.addDelay(3.0);
    }

    public void placeSamples(){
        robot.followPath(new Point(24,72, Point.CARTESIAN), new Point(24,120,Point.CARTESIAN), 0,0, false);
        afterYellow();
        robot.followPath(new Point(24,130,Point.CARTESIAN), Math.PI*3/4, 0,false);
        afterYellow();
        robot.followPath(new Point(24,140, Point.CARTESIAN),Math.PI*3/4, -Math.PI/2, false);
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
