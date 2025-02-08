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
    public static double X1 = 9, X2 = 27, X3= 12.8, X4= 28, X6=31, SX1=58.9443447037702, SX2=43.949730700179536, SX3=60.7, SX4=19.1310592459605,SX5=44.07899461400359, SX6=34.77199281867145;
    public static double Y1 = 125, Y2 = 123.5, Y3 = 129.5, Y4=129, Y6=128.5, SY1=94.23339317773788, SY2=118.79353680430879, SY3=106, SY4=130.6858168761221, SY5=117.37163375224418, SY6=118.53500897666068;
    public static double H1 = -70, H2 = -15, H3 = -40, H4 = 0, H6=35, H7 = -30;
    public static double EXT1 = 9, EXT2 = 9, EXT3 = 9, CLOSE_DELAY = 0.1, DROP_DELAY_1 = 0.7, DROP_DELAY_2= 0.7, DROP_DELAY_3 = 0.6, DOWN_DELAY = 0.65, GRABDROP_DELAY = 0.25;

    public BL04(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(7, 112, Math.toRadians(-90)));
        robot.update();
        robot.update();
    }
    public void initPos(){
        robot.arm.goTo(0,90);
    }

    public void nonSubCycles() {
//      Preload
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(0.45);
        robot.followPath(new Point(X3, Y3, Point.CARTESIAN), Math.toRadians(-90), Math.toRadians(H3), false,0.85);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
//      First Grab
        robot.followPath(new Point(X2, Y2, Point.CARTESIAN), Math.toRadians(H3), Math.toRadians(H2), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(EXT1,5, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(1, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT1, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      First Drop
        robot.queuer.addDelay(DROP_DELAY_1+GRABDROP_DELAY);
        robot.followPath(new Point(X3, Y3, Point.CARTESIAN), Math.toRadians(H2), Math.toRadians(H3), false, 0.8);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+.3);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
//      Second Grab
        robot.followPath(new Point(X4, Y4, Point.CARTESIAN), Math.toRadians(H3), Math.toRadians(H4), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(EXT2,5, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(0, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT2, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      Second Drop
        robot.queuer.addDelay(DROP_DELAY_2+GRABDROP_DELAY);
        robot.followPath(new Point(X3, Y3, Point.CARTESIAN), Math.toRadians(H4), Math.toRadians(H3), false,0.8);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+.3);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.OPEN, true);
//      Third Grab
        robot.followPath(new Point(X6, Y6, Point.CARTESIAN), Math.toRadians(H3), Math.toRadians(H6), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(EXT3,5, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(0.3, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT3, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      Third Drop
        robot.queuer.addDelay(DROP_DELAY_3+GRABDROP_DELAY);
        robot.followPath(new Point(X3, Y3, Point.CARTESIAN), Math.toRadians(H6), Math.toRadians(H3), false,0.9);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+.3);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.OPEN, true);
    }
    public void autoGrahCycle(){
        robot.followPath(new Point(SX1, SY1, Point.CARTESIAN), new Point(SX2, SY2,1), false, false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.autoGrab(2);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.followPath(new Point(SX4, SY4, Point.CARTESIAN), new Point(SX5, SY5,1), false, true);
        robot.setArm(0, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(.3);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(0);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(0);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.queuer.addDelay(0.3);
        robot.queuer.queue(false, true);

    }


    public void update() {
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}