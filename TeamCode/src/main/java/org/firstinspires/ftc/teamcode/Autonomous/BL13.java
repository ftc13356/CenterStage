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
    public static double extraX=0;
    public static double extraY=0;
    public static double extraExtraX=0;
    public static double extraExtraY=0;
    boolean shouldPark = false;
    int yellowNum =1;

    public BL13(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);
        robot.follower.setStartingPose(new Pose(7.5,87,0));
    }

    public void placeSample(){
        robot.followPath(new Point(17.5+extraExtraX,130+extraExtraY,Point.CARTESIAN), 0, -PI/4,false);
        robot.setArm(TelescopicArm.ArmStates.RETRACTED, true);
        robot.queuer.addDelay(0.7);
        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.waitForFinish();
        robot.setFlip(Flip.FlipStates.BUCKET, false);
        robot.setClaw(Claw.ClawStates.OPEN, false);
        robot.queuer.addDelay(0.5);
        robot.setFlip(Flip.FlipStates.RESET, false);
        if(!shouldPark){
            grabYellow();
        } else {
            park();
        }
    }

    public void grabYellow(){
        switch(yellowNum){
            case 1:
                robot.followPath(new Point(35+extraX,122.5+extraY, Point.CARTESIAN), 0,0, false);
                yellowNum=2;
                shouldPark = true;
                break;
            case 2:
                robot.followPath(new Point(24,130,Point.CARTESIAN), Math.PI*3/4, 0,false);
                yellowNum=3;
                break;
            case 3:
                robot.followPath(new Point(24,140, Point.CARTESIAN),Math.PI*3/4, -Math.PI/2, false);
                shouldPark=true;
                //needs to be vertical
        }
        double height=4, length=17;
        double ext = length-7, rot = 180/PI *Math.atan2(height, length);
        robot.queuer.addDelay(1.5);
        robot.setArm(ext, 0, true);
        robot.queuer.addDelay(1.0);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.waitForFinish();
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        placeSample();
    }

    public void placeSpeci(){
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, false);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN,true);
        robot.followPath(new Point(37.5,78,Point.CARTESIAN), 0,0,false);
        robot.autoReset(false);
    }

    public void park() {
        robot.followPath(new Point(74,125,Point.CARTESIAN), new Point(72,100, Point.CARTESIAN),-Math.PI*1/4, -Math.PI/2, false);
        robot.setArm(TelescopicArm.ArmStates.RETRACTED, true);
    }

    public void update() {
        robot.queuer.setFirstLoop(false);
        yellowNum = 1;
        shouldPark = false;
        robot.update();
    }
    }
