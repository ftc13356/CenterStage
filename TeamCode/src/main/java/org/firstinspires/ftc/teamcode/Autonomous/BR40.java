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
        robot.queuer.addDelay(0.7);
        robot.followPath(new Point(42.8+x3,64,Point.CARTESIAN), 0,0,false,false);
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, true);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN,true);
    }
    public void cycleBlueGrab(int i){
        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(28.1+x2,32.75+y2, Point.CARTESIAN),0, 0, false,false);
        robot.autoReset(true);
        robot.queuer.addDelay(0.5);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.queuer.addDelay(0.4);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.addDelay(0.4);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB,true);
        robot.queuer.addDelay(0.3);
        robot.followPath(new Point(22.1+x2,29.75+y2, Point.CARTESIAN),0, 0, false,false);
        robot.setClaw(Claw.ClawStates.CLOSED,false);
    }
    public void grabBlues(){
        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(31,18+y3, Point.CARTESIAN),0, 0, false);
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
        robot.followPath(new Point(31.5,5, Point.CARTESIAN),0, 0, false);
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
        robot.followPath(new Point(28+x1,17.9+y1, Point.CARTESIAN),0, 0, false);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB,true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR,true);
        robot.queuer.addDelay(1.0);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(21.2+x2,18.9+y2, Point.CARTESIAN),0, 0, false,false);
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
        robot.followPath(new Point(43.5+i/4.5+x4,64+i,Point.CARTESIAN), 0,0,false,false);
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