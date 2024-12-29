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
public class BR40Sweep {
    IDRobot robot;
    public static double x1 = 0, x2 = 0, x3 = 0, x4 = 0, x5 = 0, x6 = 0, x7 = 0, x8 = 0, x9 = 0, x10=0;
    public static double y1 = 0, y2 = 0, y3 = 0, y4 = 0, y5 = 0, y6 = 0, y7 = 0, y8 = 0, y9 = 0, y10=0;
    public static double extra = 0;
    boolean shouldPark = false;
    Pose current;

    Point sample1 = new Point(46.25+x1,22.5+y1,Point.CARTESIAN);
    Point sample2 = new Point(46.25+x2, 12.75+y2, Point.CARTESIAN);
    Point sample3 = new Point(46.25+x3, 2.5+y3, Point.CARTESIAN);
    double sample1theta, sample2theta;

    public BR40Sweep(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(7.5, 64, 0));
    }

    public void placeSpeci() {
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, false);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.followPath(new Point(39.3, 64, Point.CARTESIAN), 0, 0, false, false);
    }

    public void cycleBlueGrab(int i) {
        robot.autoReset(false);
        robot.queuer.waitForFinish();
        robot.followPath(new Point(26.1 + x8, 38.4 + y8, Point.CARTESIAN), 0, 0, false, false);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, false);
        robot.queuer.addDelay(0.4);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.addDelay(0.4);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.waitForFinish();
        robot.followPath(new Point(19.5 + x9, 37.5 + y9, Point.CARTESIAN), 0, 0, false, false);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.queuer.addDelay(.4);
    }

    public void grabBlues() {
        robot.autoReset(false);
        sample1theta=-Math.atan2(Math.abs(sample1.getY()- 30), Math.abs(sample1.getX()-26));
        robot.followPath(new Point(30,26, Point.CARTESIAN),0, sample1theta, false);
        robot.setArm(TelescopicArm.ArmStates.INTAKE, false);
        robot.followPath(new Point(current.getX(), current.getY(), Point.CARTESIAN), sample1theta, -Math.PI+sample1theta, false);

        sample2theta= -Math.atan2(Math.abs(sample2.getY()- 30), Math.abs(sample2.getX()-16));
        //prob make a new follow path that only asks for end heading and gets curent heaidng 4 u
        robot.followPath(new Point(30,16, Point.CARTESIAN),-Math.PI+sample1theta, sample2theta , false);
        robot.setArm(TelescopicArm.ArmStates.INTAKE, false);
        robot.followPath(new Point(current.getX(), current.getY(), Point.CARTESIAN), sample2theta, -Math.PI+sample2theta, false);

        robot.followPath(new Point(48,19, Point.CARTESIAN), -Math.PI+sample2theta, -Math.PI, false);
        robot.followPath(new Point(24,19, Point.CARTESIAN), -Math.PI, 0, false);

        robot.followPath(new Point(29 + x10, 35 + y10, Point.CARTESIAN),0,0, false);
        robot.followPath( new Point(22 + x5, 36.5 + y5, Point.CARTESIAN), 0, 0, false, false);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.addDelay(0.4);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
    }

    public void placeSpeci2(int i) {
        robot.followPath(new Point(20, 64 + i, 1), 0, 0, false, false);
        robot.queuer.addDelay(0.4);
        robot.setArm(0, 0, true);
        robot.queuer.addDelay(0.65);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.queuer.addDelay(0.65);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, false);
        robot.followPath(new Point(39 + x7, 64 + i, Point.CARTESIAN), 0, 0, false, false);
    }

    public void park() {
        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(20, 20, 1), 0, 0, false, false);
        robot.autoReset(true);
        robot.queuer.waitForFinish();
        robot.setArm(0, 0, true);
    }

    public void update() {
        shouldPark = false;
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}