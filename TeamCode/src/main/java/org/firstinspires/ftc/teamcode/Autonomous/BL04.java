package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHBUCKET_EXTEND_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHBUCKET_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_PITCH_POS;

import static java.lang.Math.toRadians;

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
    public static double X1 = 9, X2 = 26.5, X3= 12.5, X4= 27, X6=30, SX1=57.9443447037702, SX2=57.949730700179536, SX3=60.7, SX4=12.5310592459605,SX5=51.70556552962298, SX6=34.77199281867145;
    public static double Y1 = 125, Y2 = 122.5, Y3 = 128, Y4=128.25, Y6=127, SY1=99.23339317773788, SY2=120.79353680430879, SY3=106, SY4=127.8858168761221, SY5=119.6983842010772, SY6=118.53500897666068;
    public static double H1 = -70, H2 = -15, H3 = -30, H4 = 0, H6=35, H7 = -30;
    public static double EXT1 = 9, EXT2 = 9, EXT3 = 10, CLOSE_DELAY = 0.00, DROP_DELAY_0 = 0.2, DROP_DELAY_1 = 0.65, DROP_DELAY_2= 0.65, DROP_DELAY_3 = 0.55
            , DOWN_DELAY = 0.7, GRABDROP_DELAY = 0.1;

    public BL04(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(7, 112, toRadians(-90)));
        robot.update();
        robot.update();
    }
    public void initPos(){
        robot.arm.goTo(0,90);
    }

    public void nonSubCycles() {
//      Preload
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(DROP_DELAY_0);
        robot.followPath(new Point(X3-1, Y3+.4, Point.CARTESIAN), toRadians(-90), toRadians(H3), false,0.8);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
//        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(1, true);
//      First Grab
        robot.followPath(new Point(X2, Y2, Point.CARTESIAN), toRadians(H3), toRadians(H2), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(EXT1,2, true);
        robot.queuer.addDelay(0.1);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(1, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT1, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      First Drop
        robot.queuer.addDelay(DROP_DELAY_1+GRABDROP_DELAY);
        robot.followPath(new Point(X3, Y3, Point.CARTESIAN), toRadians(H2), toRadians(H3), false, 0.8);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+.3);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
//        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(1, true);
//      Second Grab
        robot.followPath(new Point(X4, Y4, Point.CARTESIAN), toRadians(H3), toRadians(H4), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(EXT2,2, true);
        robot.queuer.addDelay(0.1);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(0, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT2, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      Second Drop
        robot.queuer.addDelay(DROP_DELAY_2+GRABDROP_DELAY);
        robot.followPath(new Point(X3, Y3, Point.CARTESIAN), toRadians(H4), toRadians(H3), false,0.8);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+.3);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
//        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(1, true);
//      Third Grab
        robot.followPath(new Point(X6, Y6, Point.CARTESIAN), toRadians(H3), toRadians(H6), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(EXT3,3, true);
        robot.queuer.addDelay(0.1);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(0.2, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT3, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY+.2);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      Third Drop
        robot.queuer.addDelay(DROP_DELAY_3+GRABDROP_DELAY+.1);
        robot.followPath(new Point(X3, Y3, Point.CARTESIAN), toRadians(H6), toRadians(H3), false,0.85);
        robot.queuer.addDelay(GRABDROP_DELAY+.2);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+.5);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+.2);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY+.2);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(1, true);
    }
    public void autoGrahCycle(double offset){
        robot.followPath(new Point(SX2+offset, SY2,1),new Point(SX1+offset, SY1, Point.CARTESIAN), toRadians(-20), toRadians(-90), false, false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(0.1);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.autoGrab(2);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.followPath( new Point(SX5+offset, SY5,1),new Point(SX4, SY4, Point.CARTESIAN), toRadians(-80), toRadians(-30), false ,false,.9);
        robot.setArm(0, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(.3);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(0);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(0);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(1, true);

    }


    public void update() {
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}