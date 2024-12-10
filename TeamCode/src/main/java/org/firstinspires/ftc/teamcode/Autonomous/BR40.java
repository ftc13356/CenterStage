package org.firstinspires.ftc.teamcode.Autonomous;

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
    public static double x1=0, x2=0, x3=0, x4=0;
    public static double y1=0,y2=0, y3=0, y4=0;
    public static double extra=0;
    boolean shouldPark = false;
    Pose current;

    public BR40(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);
        robot.follower.setStartingPose(new Pose(7.5,64,0));
    }

    public void placeSpeci(){
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, false);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN,true);
        robot.followPath(new Point(36.5,64,Point.CARTESIAN), 0,0,false);
        robot.autoReset(false);
    }
    public void grabBlues(){
        robot.followPath(new Point(31+x1,18.5+y1, Point.CARTESIAN),0, 0, false);
        robot.queuer.addDelay(1.5);
        robot.setArm(10.25, 0, true);
        robot.queuer.addDelay(1.0);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.waitForFinish();
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.followPath(new Point(28+x2,17+y2, Point.CARTESIAN),0, 0, false);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.setFlip(Flip.FlipStates.RESET,true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR,true);
        robot.queuer.addDelay(0.2);
        robot.setClaw(Claw.ClawStates.OPEN, false);
        robot.setArm(12.5, 0, false);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE,true);
        robot.setTwist(Twist.TwistStates.PARALLEL,true);
        robot.queuer.addDelay(0.4);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        //idt the coords after this r correct
        robot.followPath(new Point(28+x3,16.25+y3, Point.CARTESIAN),0, 0, false);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.setFlip(Flip.FlipStates.RESET,true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR,true);
        robot.queuer.addDelay(1.1);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.followPath(new Point(20.75+x4,18.75+y4, Point.CARTESIAN),0, 0, false);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB,true);
        robot.setClaw(Claw.ClawStates.CLOSED,false);
    }
    public void placeSpeci2(){
        robot.queuer.addDelay(0.3);
        robot.followPath(new Point(20,64, 1), new Point(36.5,64,Point.CARTESIAN), 0,0,false);
        robot.setArm(TelescopicArm.ArmStates.RETRACTED,true);
        robot.queuer.addDelay(0.6);
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, true);
        robot.queuer.addDelay(0.6);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.queuer.addDelay(0.6);
        robot.setFlip(Flip.FlipStates.SPECIMEN,true);
        robot.autoReset(false);
    }

    public void update() {
        shouldPark= false;
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}