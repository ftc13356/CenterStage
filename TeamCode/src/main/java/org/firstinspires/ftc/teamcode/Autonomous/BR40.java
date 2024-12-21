package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_PITCH_POS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
public class BR40 {
    IDRobot robot;
    public static double x1=0, x2=0, x3=0, x4=0,x5=0,x6=0,x7=0,x8=0;
    public static double y1=0,y2=0, y3=0, y4=0,y5=0,y6=0,y7=0,y8=0;
    public static double extra=0;
    boolean shouldPark = false;
    Pose current;

    public BR40(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);
        robot.follower.setStartingPose(new Pose(7.5,64,0));
    }

    public void placeSpeci(){
        robot.queuer.addDelay(0.6);
        robot.followPath(new Point(40.3+x1,64,Point.CARTESIAN), 0,0,false,false);
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, true);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN,true);
    }
    public void cycleBlueGrab(int i){
        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(26.1+x7,38+y7, Point.CARTESIAN),0, 0, false,false);
        robot.autoReset(true);
        robot.queuer.addDelay(0.5);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.queuer.addDelay(0.4);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.addDelay(0.4);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB,true);
        robot.queuer.addDelay(0.3);
        robot.followPath(new Point(20.3+x8,35+y8, Point.CARTESIAN),0, 0, false,false);
        robot.setClaw(Claw.ClawStates.CLOSED,false);
    }
    public void grabBlues(){
        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(29+x2,23+y2, Point.CARTESIAN),0, 0, false);
        robot.autoReset(true);
        robot.queuer.addDelay(1.2);
        robot.setArm(2, 0, true);
        robot.queuer.addDelay(2);
        robot.setArm(12.7, 0, true);
        robot.queuer.addDelay(1.4);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.queuer.addDelay(1.4);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.waitForFinish();
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.followPath(new Point(28.5+x3,11+y3, Point.CARTESIAN),0, 0, false);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN,true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR,true);
        robot.queuer.addDelay(0.9);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.queuer.waitForFinish();
        robot.setArm(11.45, 0, false);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE,true);
        robot.setTwist(Twist.TwistStates.PARALLEL,true);
        robot.queuer.addDelay(0.4);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.queuer.addDelay(0.3);
        robot.followPath(new Point(28+x4,22.9+y4, Point.CARTESIAN),0, 0, false);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN,true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR,true);
        robot.queuer.addDelay(1.0);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(20.7+x5,23.9+y5, Point.CARTESIAN),0, 0, false,false);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB,true);
        robot.setClaw(Claw.ClawStates.CLOSED,false);
    }
    public void placeSpeci2(int i){
        robot.queuer.addDelay(0.7);
        robot.followPath(new Point(20,64+i,1),0,0,false,false);
        robot.queuer.addDelay(0.4);
        robot.setArm(0,0,true);
        robot.queuer.addDelay(0.65);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.queuer.addDelay(0.65);
        robot.setFlip(Flip.FlipStates.SPECIMEN,true);
        robot.queuer.addDelay(0.35);
        robot.followPath(new Point(39+x6,64+i,Point.CARTESIAN), 0,0,false,false);
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, true);
    }
    public void park(){
        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(20,20,1),0,0,false,false);
        robot.autoReset(true);
        robot.queuer.addDelay(0.8);
        robot.setArm(0,0,true);
    }

    public void update() {
        shouldPark= false;
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}