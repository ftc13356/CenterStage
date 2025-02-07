package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHBUCKET_EXTEND_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHBUCKET_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_PITCH_POS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
@Config
public class BL04 {
    IDRobot robot;
    public static double X1 = 9, X2 = 22, X3= 20, X4= 25, X6=29, X7=15.5, X8=68;
    public static double Y1 = 125, Y2 = 126, Y3 = 135, Y4=130, Y6=130, Y7=132.2, Y8=94.6;
    public static double H1 = -70, H2 = -15, H3 = -15, H4 = 0, H6=35, H7 = -30;
    public static double EXT1 = 10, EXT2 = 10, EXT3 = 10, CLOSE_DELAY = 0.2;

    public BL04(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(7, 112, Math.toRadians(-90)));
        robot.update();
        robot.update();
    }

    public void nonSubCycles() {
//      Preload
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(0.5);
        robot.followPath(new Point(X1, Y1, Point.CARTESIAN), Math.toRadians(-90), Math.toRadians(H1), false);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
//      First Grab
        robot.followPath(new Point(X2, Y2, Point.CARTESIAN), Math.toRadians(H1), Math.toRadians(H2), false);
        robot.setArm(EXT1,6, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(1, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT1, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      First Drop
        robot.followPath(new Point(X3, Y3, Point.CARTESIAN), Math.toRadians(H2), Math.toRadians(H3), false);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
//      Second Grab
        robot.followPath(new Point(X4, Y4, Point.CARTESIAN), Math.toRadians(H3), Math.toRadians(H4), false);
        robot.setArm(EXT2,6, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(0, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT2, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      Second Drop
        robot.followPath(new Point(X3, Y3, Point.CARTESIAN), Math.toRadians(H4), Math.toRadians(H3), false);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.OPEN, true);
//      Third Grab
        robot.followPath(new Point(X6, Y6, Point.CARTESIAN), Math.toRadians(H3), Math.toRadians(H6), false);
        robot.setArm(EXT3,6, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(0.3, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT3, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      Third Drop
        robot.followPath(new Point(X7, Y7, Point.CARTESIAN), Math.toRadians(H6), Math.toRadians(H7), false);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.OPEN, true);
    }


    public void update() {
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}