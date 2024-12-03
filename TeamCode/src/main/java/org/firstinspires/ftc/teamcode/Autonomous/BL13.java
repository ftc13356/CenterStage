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
    PathChain scoreSpecy, scoreSampley, getSampley1, getSampley2, getSampley3, submersible;
    IDRobot robot;

    public BL13(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);

        robot.follower = new Follower(opmode.hardwareMap);
        Pose starting = new Pose(12,108,0);
        robot.follower.setStartingPose(starting);

        scoreSpecy = robot.follower.pathBuilder()
                .addPath(new BezierCurve(new Point(12,108,Point.CARTESIAN), new Point(36,72,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(0,0)
                .build();

        getSampley1 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(new Point(36,72,Point.CARTESIAN), new Point(24,72,Point.CARTESIAN), new Point(24,120,0)))
                .setLinearHeadingInterpolation(0,0)
                .build();


        getSampley2 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(24,130,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4, 0)
                .build();

        getSampley3 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(24,140, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4, 0)
                .build();

        submersible = robot.follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(60,110,Point.CARTESIAN), new Point(60,96,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4, -Math.PI/2)
                .build();
    }
    /**
     * builds + follows path to go from current point -> sample-scoring
     */
    public void placeSample () {
        Point score = new Point(20, 124, Point.CARTESIAN);
        robot.followPath(score, 0, Math.PI/4,true,true);

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
        robot.followPath(scoreSpecy,true);
        robot.queuer.addDelay(3.0);
    }

    /**
     * follows paths + sets claw and arm to pick up and places yellow samples 1,2,3
     */

    public void afterYellow(){
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.setArm(TelescopicArm.ArmStates.INTAKE, true);
        robot.queuer.addDelay(3.0);

        placeSample();
        robot.setClaw(Claw.ClawStates.CLOSED, true);
        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, true);
        robot.queuer.addDelay(3.0);
    }

    public void placeSamples(){
        robot.followPath(getSampley1, true);
        afterYellow();
        robot.followPath(getSampley2,true);
        afterYellow();
        robot.followPath(getSampley3,true);
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
