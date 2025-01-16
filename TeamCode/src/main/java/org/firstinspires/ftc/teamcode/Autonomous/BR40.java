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
    public static double x1 = 0, x2 = 0, x3 = 0, x4 = 0, x5 = 0, x6 = 0, x7 = 0;
    public static double x8 = 0, x9 = 0, x10=0, x12=0;
    public double  y2 = 0, y3 = 0, y4 = 0, y5 = 0, y6 = 0, y7 = 0, y8 = 0, y9 = 0, y10=0,DROP_DELAY = .9;
    boolean shouldPark = false;

    public BR40(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(7.5, 64, 0));
        robot.update();
        robot.update();
    }

    public void placeSpeci() {
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, false);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.addDelay(.65);
        robot.followPath(new Point(39.3 + x1, 64, Point.CARTESIAN), 0, 0, false, false);
    }

    public void cycleBlueGrab(int i) {
//        robot.autoReset(false);
        robot.setArm(TelescopicArm.ArmStates.RETRACTED, true);
        robot.followPath(new Point(25.8 + x8, 38.25 + y8, Point.CARTESIAN),new Point(19 + x9, 35.75 + y9, Point.CARTESIAN), 0,0, false);
//        robot.followPath(new Point(25.8 + x8, 38.25 + y8, Point.CARTESIAN), 0, 0, false, false);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.queuer.addDelay(0.2);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.addDelay(0.2);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.addDelay(0.2);
//        robot.followPath(new Point(19 + x9, 35.75 + y9, Point.CARTESIAN), 0, 0, false, false);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.queuer.addDelay(0.5);
    }

    public void grabBlues() {
//        robot.autoReset(false);
        robot.followPath(new Point(27.7 + x2, 22.55 + y2, Point.CARTESIAN), 0, 0, false);
//        robot.setArm(14.7, 0, false);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
//        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.followPath(new Point(25.15 + x3, 13.05 + y3, Point.CARTESIAN), 0, 0, false);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.waitForFinish();
        robot.setClaw(Claw.ClawStates.OPEN, false);
//        robot.setArm(16.95, 0, false);
        robot.queuer.addDelay(0.5);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.addDelay(0.5);

        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.queuer.waitForFinish();
        robot.setClaw(Claw.ClawStates.CLOSED, false);
//        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, false);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.waitForFinish();
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(29 + x10, 35 + y10, Point.CARTESIAN),0,0, false);
        robot.followPath( new Point(23 + x5, 30 + y5, Point.CARTESIAN), 0, 0, false, false);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.addDelay(0.4);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
    }

    public void grabBluesSweep(){
        //reset
        //grab1
        robot.autoReset(false);
        robot.followPath(new Point(38.5+x3,31.25+y3, Point.CARTESIAN),0, Math.toRadians(-40), false);
        robot.autoReset(true);
        robot.queuer.addDelay(0.75);
        robot.setArm(7.5+x1,10,true);
//        robot.queuer.addDelay(0.5);
        robot.queuer.addDelay(0.2);
        robot.setTwist(0.83+x12, true);
        robot.queuer.addDelay(0.2);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setArm(7.5+x1,0,false);
//        robot.queuer.waitForFinish();
        robot.queuer.addDelay(0.1);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.queuer.addDelay(0.2);
        //drop1
        robot.followPath(new Point(22.5, 30, Point.CARTESIAN), Math.toRadians(-40), -3*Math.PI/4, false,0.9);
        robot.setArm(6,5,true);
        robot.queuer.addDelay(DROP_DELAY);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        //grab2
        robot.followPath(new Point(31.25+x4,24+y4, Point.CARTESIAN),-3*Math.PI/4, Math.toRadians(-40) , false);
        robot.setTwist(0.83+x12, true);
        robot.setArm(8+x2,10,true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setArm(8+x2,0,false);
        robot.queuer.addDelay(0.1);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.queuer.addDelay(0.2);
        //drop2
        robot.followPath(new Point(23.5+x7, 22+y7, Point.CARTESIAN), Math.toRadians(-40), -3*Math.PI/4, false,0.9);
        robot.setArm(5,5,true);
        robot.queuer.addDelay(DROP_DELAY);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        //grab3
        robot.followPath(new Point(31+x8,17+y8, Point.CARTESIAN), -3*Math.PI/4, Math.toRadians(-40), false);
        robot.setArm(10+x5,10,true);
        robot.setTwist(0.83+x12, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setArm(10+x5,0,false);
        robot.queuer.addDelay(0.1);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.queuer.addDelay(0.7);
        //drop3
        robot.followPath(new Point(29 +x9,22+y9, Point.CARTESIAN), -Math.PI/4, 0, false);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB,true);
        robot.queuer.addDelay(0.4);
        robot.setClaw(Claw.ClawStates.OPEN, false);
        //grab4
        robot.followPath(new Point(26.75, 34, Point.CARTESIAN), new  Point(23.5, 35.5, Point.CARTESIAN),0,0, false);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.queuer.addDelay(0.4);
    }

    public void placeSpeci2(int i) {
        robot.followPath(new Point(21, 64 + i, 1),new Point(39.75 + x7, 64 + i, Point.CARTESIAN), 0,0, false);
//        robot.followPath(new Point(21, 64 + i, 1), 0, 0, false, 0.9);
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, true);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
//        robot.followPath(new Point(39.75 + x7, 64 + i, Point.CARTESIAN), 0, 0, false, false);
        robot.setClaw(Claw.ClawStates.OPEN, false);
    }

    public void park() {
        robot.followPath(new Point(20, 20, 1), 0, 0, false, false);
        robot.autoReset(true);
        robot.setArm(0, 0, true);
    }

    public void update() {
        shouldPark = false;
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}