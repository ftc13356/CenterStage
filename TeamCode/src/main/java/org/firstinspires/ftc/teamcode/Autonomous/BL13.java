//ignore this + BlueLeft31

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
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
public class BL13 {
    IDRobot robot;
    public static double x=0;
    public static double y=0;
    boolean shouldPark = false;
    int yellowNum =1;

    public BL13(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);
        robot.follower.setStartingPose(new Pose(7.5,103,0));
    }

    public void placeSample(){
        robot.followPath(new Point(16,126,Point.CARTESIAN), 0, -PI/4,false);
        robot.queuer.addDelay(0.7);
        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, false);
        robot.queuer.waitForFinish();
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.BUCKET, true);
        robot.queuer.addDelay(0.5);
        robot.setClaw(Claw.ClawStates.OPEN, false);
        robot.queuer.addDelay(1.0);
        robot.setFlip(Flip.FlipStates.RESET, false);

        Pose current = robot.follower.getPose();
        robot.queuer.addDelay(1.0);
        robot.autoReset(false);
        robot.followPath(new Point(current.getX()+2, current.getY()-2, Point.CARTESIAN), -PI/4, -PI/4, true);

        if(!shouldPark){
            grabYellow();
        } else {
            park();
        }
    }

    public void grabYellow(){
        switch(yellowNum){
            case 1:
                robot.followPath(new Point(24,117, Point.CARTESIAN), new Point(24,120,Point.CARTESIAN), 0,0, false);
                yellowNum=2;
                break;
            case 2:
                robot.followPath(new Point(24,130,Point.CARTESIAN), Math.PI*3/4, 0,false);
                yellowNum=3;
                break;
            case 3:
                robot.followPath(new Point(24,140, Point.CARTESIAN),Math.PI*3/4, -Math.PI/2, false);
                shouldPark=true;
        }
        double height=4, length=15;
        double ext = length-7, rot = 180/PI *Math.atan2(height, length);
        robot.queuer.addDelay(1.0);
        robot.setArm(ext, 0, true);
        //robot.setArm(ext, rot, true);
        robot.queuer.addDelay(1.0);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        placeSample();
    }

    public void placeSpeci(){
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(3.0);
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, false);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN,true);
        robot.followPath(new Point(36,72,Point.CARTESIAN), 0,0,false);
        robot.autoReset(false);
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