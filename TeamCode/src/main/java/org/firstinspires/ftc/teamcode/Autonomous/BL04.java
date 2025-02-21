package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHBUCKET_EXTEND_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHBUCKET_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Math.min;
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
    public IDRobot robot;
    public static double X1 = 9, X2 = 27, X3= 13.25, X4= 26, X6=28, SX1=57.9443447037702, SX2=57.949730700179536, SX3=60.7, SX4=8.0310592459605,SX5=51.70556552962298, SX6=34.77199281867145;
    public static double Y1 = 125, Y2 = 120.5, Y3 = 130, Y4=127.5, Y6=127.3, SY1=103.73339317773788, SY2=122.79353680430879, SY3=106, SY4=129.5858168761221, SY5=119.6983842010772, SY6=118.53500897666068;
    public static double H1 = -70, H2 = -11, H3 = -30, H4 = 0, H6=35, H7 = -30;
    public static double EXT1 = 8.5, EXT2 = 9.5, EXT3 = 12.8, CLOSE_DELAY = 0.00, DROP_DELAY_0 = 0.3, DROP_DELAY_1 = 0.6, DROP_DELAY_2= 0.6, DROP_DELAY_3 = 0.5
            , DOWN_DELAY = 0.5, EXT_DELAY = 0.3, OTHER_EXT_DELAY = 0.35, GRABDROP_DELAY = 0.1, STRAFE_DELAY = 0.0;
    int[] positions = {0,0,0,0};
    int ind = 0;
    boolean isDroppi= false;

    public BL04(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(6.7, 112, toRadians(-90)));
        robot.update();
        robot.update();
        isDroppi = false;
    }
    public void initLoop(){
        robot.arm.goTo(0,90);
        boolean isRD = gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "auto grab red");
        boolean isLD = gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "auto grab red");
        boolean isUD = gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "auto grab red");
        boolean isDD = gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "isRed");
        if(isLD){
            ind = Math.max(0, ind-1);
        }
        if(isRD){
            ind = Math.min(3, ind+1);
        }
        if(isUD){
            for(int i=ind; i<4; i++){
                positions[i]+=1;
            }
        }
        if(isDD){
            for(int i=ind; i<4; i++){
                positions[i]-=1;
            }
        }
        for (int i=0;i<4; i++){
            packet.put("position"+i,positions[i]);
            op.telemetry.addData("position"+i,positions[i]);
        }
        robot.update();
        op.telemetry.update();
    }

    public void nonSubCycles() {
//      Preload
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(DROP_DELAY_0);
        robot.followPath(new Point(X3-3, Y3-2, Point.CARTESIAN), toRadians(-90), toRadians(-30), false,0.98);
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
        robot.setArm(0,3, true);
        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY);
        robot.setArm(EXT1,3, true);
        robot.queuer.addDelay(0.1);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(1, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT1, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      First Drop
        robot.queuer.addDelay(DROP_DELAY_1+GRABDROP_DELAY);
        robot.followPath(new Point(X3, Y3, Point.CARTESIAN), toRadians(H2), toRadians(H3), false, 0.75);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+OTHER_EXT_DELAY);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(1, true);
//      Second Grab
        robot.followPath(new Point(X4, Y4, Point.CARTESIAN), toRadians(H3), toRadians(H4), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(0,2, true);
        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY);
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
        robot.followPath(new Point(X3-1, Y3-1, Point.CARTESIAN), toRadians(H4), toRadians(H3), false,0.75);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+OTHER_EXT_DELAY);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(1, true);
//      Third Grab
        robot.followPath(new Point(X6, Y6-1, Point.CARTESIAN), toRadians(H3), toRadians(H6), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(0,7, true);
        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY);
        robot.setArm(EXT3,2, true);
        robot.queuer.addDelay(0.1);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setTwist(0.2, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT3, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
//      Third Drop
        robot.queuer.addDelay(DROP_DELAY_3+GRABDROP_DELAY);
        robot.followPath(new Point(X3-2, Y3-1, Point.CARTESIAN), toRadians(H6), toRadians(H3), false,0.85);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+OTHER_EXT_DELAY+.2);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(1, true);
    }
    public void autoGrahCycle(int ind){
        if(robot.queuer.queue(false, true)){
            isDroppi = false;
        }
        robot.followPath(new Point(SX2, SY2,1),new Point(SX1+positions[ind]*5, SY1+positions[ind]*1, Point.CARTESIAN), toRadians(-30), toRadians(-87), false, 5, false, 0.9);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(0.1);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY);
        robot.autoGrab(2);
        robot.queuer.addDelay(STRAFE_DELAY);
        robot.followPathNotTargeted(new Point(80.9443447037702, 98.23339317773788,1), new Point(55.5, 98.23339317773788,1)
                ,0.4,toRadians(-90),toRadians(-90),true);
        robot.queuer.waitForFinish();
//        robot.queuer.addDelay(0.2);
        if(robot.queuer.queue(false, true) || !robot.queuers.get(2).isEmpty()){
            isDroppi = true;
        }
        robot.queuer.addDelay(.2);
        robot.followPath( new Point(SX5, SY5,1),new Point(SX4, SY4, Point.CARTESIAN), toRadians(-60), toRadians(-33), false ,4.7,false,.9);
        robot.queuer.addDelay(.2);
        robot.setArm(0, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(.75);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS-3, true);
        robot.queuer.addDelay(.45);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(.45);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setTwist(1, true);

    }
    public boolean isDroppi(){
        return isDroppi;
    }
    public void park(){
        robot.followPath(new Point(SX1+10, SY1+2,1 ), toRadians(-90), toRadians(-90),false, 0.8);
        robot.setArm(0,45, true);
        robot.setTwist(Twist.TwistStates.PARALLEL,true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.addDelay(0.7);
        robot.setArm(21,45,true);
    }
    public void reset(){
        robot.followPath(new Point(SX1+10, SY1+2,1 ), toRadians(-90), toRadians(-90),false, 0.8);
        robot.setArm(0,90, true);
        robot.setTwist(Twist.TwistStates.PARALLEL,true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
    }


    public void update() {
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}