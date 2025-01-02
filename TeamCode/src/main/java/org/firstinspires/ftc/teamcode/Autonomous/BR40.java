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
    public double x1 = 0, x2 = 0, x3 = 0, x4 = 0, x5 = 0, x6 = 0, x7 = 0;
    public static double x8 = 0, x9 = 0, x10=0, x11=0, x12=0, x13=0;
    public static double y1 = 0, y2 = 0, y3 = 0, y4 = 0, y5 = 0, y6 = 0, y7 = 0, y8 = 0, y9 = 0, y10=0, y11=0, y12=0, y13=0;
    public static double extra = 0;
    boolean shouldPark = false;

    public BR40(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(7.5, 64, 0));
        robot.update();
        robot.update();
        robot.update();
    }

    public void placeSpeci() {
//        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, false);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.followPath(new Point(32.3 + x1, 64, Point.CARTESIAN), 0, 0, false, false);
    }

    public void cycleBlueGrab(int i) {
//        robot.autoReset(false);
        robot.queuer.waitForFinish();
        robot.followPath(new Point(26.1 + x8, 38.4 + y8, Point.CARTESIAN), 0, 0, false, false);
//        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, false);
        robot.queuer.addDelay(0.4);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.addDelay(0.4);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.waitForFinish();
        robot.followPath(new Point(21 + x9, 36 + y9, Point.CARTESIAN), 0, 0, false, false);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.queuer.addDelay(.4);
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
        robot.followPath( new Point(23 + x5, 32 + y5, Point.CARTESIAN), 0, 0, false, false);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.addDelay(0.4);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
    }

    public void grabBluesSweep(){
            robot.autoReset(false);
            robot.followPath(new Point(35+x4,37+y4, Point.CARTESIAN),0, -Math.PI/4, false);
            robot.setArm(13+x1,5+y1, false);
            robot.setTwist(0.83+x12, true);
            robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
            robot.setArm(13+x1,0, false);
            robot.setClaw(Claw.ClawStates.CLOSED, false);
            robot.setArm(8+x2,3, false);
            robot.followPath(new Point(26+x4, 30+y5, Point.CARTESIAN), -Math.PI/4, -3*Math.PI/4, false);
            robot.setClaw(Claw.ClawStates.OPEN, false);

            robot.followPath(new Point(33.5+x4,25+y6, Point.CARTESIAN),-3*Math.PI/4, -Math.PI/4 , false);
            robot.setArm(13+x1,5+y1, false);
            robot.setTwist(0.83+x12, true);
            robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
            robot.setArm(13+x1,0, false);
            robot.setClaw(Claw.ClawStates.CLOSED, false);
             robot.setArm(7+x2,3, false);
            robot.followPath(new Point(26+x4, 20+y7, Point.CARTESIAN), -Math.PI/4, -3*Math.PI/4, false);
            robot.setClaw(Claw.ClawStates.OPEN, false);
//
            robot.setArm(4, 20+y1,true);
            robot.followPath(new Point(35.5+x8,15+y8, Point.CARTESIAN), -3*Math.PI/4, -Math.PI/4, false);
            robot.setArm(10+x2,5+y1, false);
            robot.setTwist(0.83+x12, true);
            robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
            robot.setArm(10+x1,0, false);
            robot.setClaw(Claw.ClawStates.CLOSED, false);
            robot.followPath(new Point(26 +x9,22+y9, Point.CARTESIAN), -Math.PI/4, 0, false);
            robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, false);
            robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB,true);
            robot.setClaw(Claw.ClawStates.OPEN, false);

            robot.followPath( new Point(23, 33.5, Point.CARTESIAN), 0, 0, false, false);
            robot.setTwist(Twist.TwistStates.SPECIMEN, true);
            robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
            robot.queuer.addDelay(0.4);
            robot.setClaw(Claw.ClawStates.CLOSED, false);
    }

    public void placeSpeci2(int i) {
        robot.followPath(new Point(20, 64 + i, 1), 0, 0, false, false);
        robot.queuer.addDelay(0.4);
//        robot.setArm(0, 0, true);
        robot.queuer.addDelay(0.65);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.queuer.addDelay(0.65);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.queuer.addDelay(4.0);
//        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, false);
        robot.queuer.addDelay(0.0);
        robot.followPath(new Point(39 + x7, 64 + i, Point.CARTESIAN), 0, 0, false, false);
    }

    public void park() {
        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(20, 20, 1), 0, 0, false, false);
//        robot.autoReset(true);
        robot.queuer.waitForFinish();
        robot.setArm(0, 0, true);
    }

    public void update() {
        shouldPark = false;
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}