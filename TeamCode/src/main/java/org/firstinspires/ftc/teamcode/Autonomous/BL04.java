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
    public static double X1 = 9, X2 = 28.7, X3= 21, X4= 28.4, X6=32, SX1=57.9443447037702, SX2=57.949730700179536, SX3=60.7, SX4=9.0310592459605,SX5=51.70556552962298, SX6=34.77199281867145;
    public static double Y1 = 125, Y2 = 122.75, Y3 = 126, Y4=129, Y6=129.9, SY1=101.73339317773788, SY2=122.79353680430879, SY3=106, SY4=126.6858168761221, SY5=119.6983842010772, SY6=118.53500897666068;
    public static double H1 = -70, H2 = -5, H3 = -35, H4 = 0, H6=35, H7 = -30;
    public static double EXT1 = 12, EXT2 = 12, EXT3 = 13.5, CLOSE_DELAY = 0.00, DROP_DELAY_0 = 0.0, DROP_DELAY_1 = 0.75, DROP_DELAY_2= 0.7, DROP_DELAY_3 = 0.55
            , DOWN_DELAY = 0.3, EXT_DELAY = 0.1, OTHER_EXT_DELAY = 0.4, GRABDROP_DELAY = 0.0, STRAFE_DELAY = 0.0;
    int[] positions = {0,0,0,0};
    int ind = 0;

    int isRed = 0;
    boolean isDroppi= false;

    public BL04(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(6.7, 112, toRadians(-90)));
        robot.update();
        robot.update();
        isDroppi = false;
    }
    public void initLoop(){
        robot.twisPerp();
//        robot.claw.goTo(Claw.ClawStates.CLOSED);
        robot.arm.goTo(0,90);
        boolean isRD = gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "auto grab red");
        boolean isLD = gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "auto grab red");
        boolean isUD = gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "auto grab red");
        boolean isDD = gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "isRed");
        boolean isA = gampad.readGamepad(op.gamepad1.a, "gamepad1_a", "colorSwitch");
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
        if(isA){
            if(isRed == 0){
                isRed =1;
            }else{
                isRed = 0;
            }
        }
        for (int i=0;i<4; i++){
            packet.put("position"+i,positions[i]);
            op.telemetry.addData("position"+i,positions[i]);
        }
        op.telemetry.addData("isRed", isRed);
        packet.put("isRed", isRed);
        robot.update();
        op.telemetry.update();
    }

    public void nonSubCycles() {
        if(robot.queuer.isFirstLoop()){
            if(isRed==1){
                robot.cv.swapTeleRed();
            } else{
                robot.cv.swapTeleBlue();
            }
        }
//      Preload
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(DROP_DELAY_0);
        robot.followPath(new Point(X3-3, Y3+.5, Point.CARTESIAN), toRadians(-90), toRadians(-40), false,0.99);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true);
        robot.setFlip(Flip.FlipStates.BUCKET, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(0.06);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        //      First Grab
        robot.followPath(new Point(X2, Y2-2, Point.CARTESIAN), toRadians(-40), toRadians(H2), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(0,5, true);
//        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY);
//        robot.setArm(EXT1-4,7, true);
        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY+0.2);
        robot.setArm(EXT1,7, true);
        robot.queuer.addDelay(0.2);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.addDelay(0.25);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT1, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
//      First Drop
        robot.queuer.addDelay(DROP_DELAY_1+GRABDROP_DELAY);
        robot.followPath(new Point(X3+.75, Y3-3.75, Point.CARTESIAN), toRadians(H2), toRadians(H3), false, 0.9);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+OTHER_EXT_DELAY);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS+2, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.BUCKET, true);
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(0.06);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
//      Second Grab
        robot.followPath(new Point(X4, Y4, Point.CARTESIAN), toRadians(H3), toRadians(H4), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(0,5, true);
//        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY);
//        robot.setArm(EXT2-4,7, true);
        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY+.2);
        robot.setArm(EXT2,7, true);
        robot.queuer.addDelay(0.2);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.addDelay(0.2);
        robot.setTwist(0, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT2, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
//      Second Drop
        robot.queuer.addDelay(DROP_DELAY_2+GRABDROP_DELAY);
        robot.followPath(new Point(X3+0.15, Y3-.25, Point.CARTESIAN), toRadians(H4), toRadians(H3), false,0.85);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(GRABDROP_DELAY+OTHER_EXT_DELAY);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS+2, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.BUCKET, true);
        robot.queuer.queue(false, true);
//        robot.queuer.addDelay(0.06);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.waitForFinish();
//      Third Grab
        robot.followPath(new Point(X6-.5, Y6-1, Point.CARTESIAN), toRadians(H3), toRadians(H6), false);
        robot.setArm(0,HIGHBUCKET_PITCH_POS,true);
        robot.queuer.addDelay(DOWN_DELAY);
        robot.setArm(0,5, true);
        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY);
//        robot.setArm(EXT3-8,7, true);
        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY+.2);
        robot.setArm(EXT3,7, true);
        robot.queuer.addDelay(0.2);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.addDelay(0.25);
        robot.setTwist(0.2, true);
        robot.queuer.waitForFinish();
        robot.setArm(EXT3, 0, false);
        robot.queuer.addDelay(CLOSE_DELAY);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
//      Third Drop
        robot.queuer.addDelay(DROP_DELAY_3+GRABDROP_DELAY);
        robot.followPath(new Point(X3-2, Y3+1, Point.CARTESIAN), toRadians(H6), toRadians(H3), false,0.94);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setArm(0,HIGHBUCKET_PITCH_POS-3, true);
        robot.queuer.addDelay(GRABDROP_DELAY+OTHER_EXT_DELAY);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS-2, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(GRABDROP_DELAY);
        robot.setFlip(Flip.FlipStates.BUCKET, true);
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(0.06);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.waitForFinish();

    }
    public void autoGrahCycle(int ind){
        if(robot.queuer.queue(false, true)){
            isDroppi = false;
        }
        robot.followPath(new Point(SX2, SY2,1),new Point(SX1+positions[ind]*2, SY1+positions[ind]*-1.2, Point.CARTESIAN), toRadians(-30), toRadians(-87), false, 5, false, 0.9);
        robot.queuer.addDelay(0.2);
        robot.setFlip(Flip.FlipStates.AUTO_GRAH, true);
        robot.setArm(0,HIGHBUCKET_PITCH_POS, true);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.addDelay(DOWN_DELAY+EXT_DELAY+.3);
        robot.autoGrab(2);
        robot.queuer.addDelay(STRAFE_DELAY);
        robot.followPathNotTargeted(new Point(80.9443447037702, 98.23339317773788,1), new Point(55.5, 98.23339317773788,1)
                ,0.35,toRadians(-90),toRadians(-90),true);
        robot.queuer.waitForFinish();
//        robot.queuer.addDelay(0.2);
        if(robot.queuer.queue(false, true) || !robot.queuers.get(2).isEmpty()){
            isDroppi = true;
        }
        robot.queuer.addDelay(.1);
        robot.followPath( new Point(SX5, SY5,1),new Point(SX4-ind*.4, SY4-ind*.2, Point.CARTESIAN), toRadians(-60), toRadians(-38), false ,3.5,false,0.85);
        robot.queuer.addDelay(.05);
        robot.setArm(0, HIGHBUCKET_PITCH_POS, true);
        robot.queuer.addDelay(.75);
        robot.setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS-5, true);
        robot.queuer.addDelay(.15);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(.25);
        robot.setFlip(Flip.FlipStates.BUCKET, true);
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(0.1);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.queue(false, true);
        robot.queuer.waitForFinish();
    }
    public boolean isDroppi(){
        return isDroppi;
    }
    public void park(){
        robot.followPath(new Point(SX1+10, SY1+2,1 ), toRadians(-90), toRadians(-90),false, 0.8);
        robot.setArm(0,45, true);
        robot.setTwist(Twist.TwistStates.PARALLEL,true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.queuer.addDelay(0.7);
        robot.setArm(21,45,true);
    }
    public void reset(){
//        robot.followPath(new Point(SX1+10, SY1+2,1 ), toRadians(-90), toRadians(-90),false, 0.8);
        robot.setArm(0,20, true);
        robot.setTwist(Twist.TwistStates.PARALLEL,true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
    }


    public void update() {
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}