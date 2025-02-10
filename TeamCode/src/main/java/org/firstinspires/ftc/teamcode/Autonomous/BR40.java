package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST3_X;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST3_Y;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;
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
    public static double x8 = 0, x9 = 0, x10=0, x12=0, y9 =0;
    public double  y2 = 0, y3 = 0, y4 = 0, y5 = 0, y6 = 0, y7 = 0, y8 = 0, y10=0,DROP_DELAY = 0.6;
    int position = 0;
    boolean shouldPark = false, shouldAuto = true;

    public BR40(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(7.5, 64, 0));
        robot.update();
    }
    public void initLoop(){
        boolean isRD = gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "auto grab red");
        boolean isLD = gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "auto grab red");
        boolean isUD = gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "auto grab red");
        if(isLD){
            position+=1;
        }
        if(isRD){
            position-=1;
        }
        if(isUD){
            shouldAuto = !shouldAuto;
        }
        packet.put("position",position);packet.put("shouldGrah",shouldAuto);
        op.telemetry.addData("position", position);
        op.telemetry.addData("shouldGrah", shouldAuto);

        robot.update();
        op.telemetry.update();
    }

    public void placeSpeci() {
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, true);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.followPath(new Point(39.4 + x1, 67, Point.CARTESIAN), 0, 0, false, .9);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.followPath(new Point(36.4 + x1, 67, Point.CARTESIAN), 0, 0, false, .9);
    }

    public void autoGrab() {
        if(shouldAuto) {
            robot.setArm(0, 33, false);
            robot.autoReset(false);
//        robot.followPath(new Point(36.4 + x1, 67, Point.CARTESIAN), 0, 0, false, .9);
//        robot.queuer.addDelay(.5);
            robot.followPath(new Point(42 + x1, 72 + position * 4, Point.CARTESIAN), 0, 0, true, .9);
            robot.autoGrab(1);
            robot.queuer.waitForFinish();
            Vector2d dist3 = new Vector2d(DIST3_X, DIST3_Y);
            robot.followPath(new Point(17 + dist3.getX(), 33.5 + dist3.getY(), Point.CARTESIAN), 0, 0, false, false);
            robot.setArm(0, 15, true);
            robot.queuer.addDelay(0.3);
            robot.setFlip(Flip.FlipStates.RESET, true);
            robot.queuer.addDelay(0.3);
            robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
            robot.queuer.addDelay(0.25);
            robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
            robot.queuer.addDelay(1.5);
            robot.setClaw(Claw.ClawStates.OPEN, true);
        }
        else{
            cycleBlueGrab(2);
        }
    }

    public void grabBlues() {
//        robot.autoReset(false);
        robot.followPath( new Point(27.7 + x2, 22.55 + y2, Point.CARTESIAN), 0, 0, false);
//        robot.setArm(14.7, 0, false);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
//        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.followPath(new Point(19.15 + x3, 19.05 + y3, Point.CARTESIAN), 0, 0, false);
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
        //grab1
        robot.autoReset(false);
        robot.followPath(new Point(29.5+x3,34.85+y3, Point.CARTESIAN),0, Math.toRadians(-40), false, .8);
        robot.queuer.addDelay(0.85);
        robot.setArm(11.5+x1,10,true);
//        robot.queuer.addDelay(0.2);
        robot.setTwist(0.83+x12, true);
        robot.queuer.addDelay(0.2);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setArm(11.5+x1,0,false);
        robot.queuer.addDelay(0.2);
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
        //drop1
        robot.followPath(new Point(29.5, 32, Point.CARTESIAN), -3*Math.PI/4, -3*Math.PI/4, false,0.85);
        robot.setArm(13,5,true);
        robot.queuer.addDelay(DROP_DELAY);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        //grab2
        robot.followPath(new Point(31+x4,26.85+y4, Point.CARTESIAN),Math.toRadians(-44), Math.toRadians(-44) , false, .8);
        robot.setTwist(0.83+x12, true);
        robot.setArm(11,5,true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
//        robot.queuer.addDelay(0.2);
        robot.setArm(11,0,false);
        robot.queuer.addDelay(0.2);
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
        //drop2
        robot.followPath(new Point(29.5+x7, 21+y7, Point.CARTESIAN), -3*Math.PI/4, -3*Math.PI/4, false,0.9);
        robot.setArm(13,5,true);
        robot.queuer.addDelay(DROP_DELAY);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        //grab3
        robot.followPath(new Point(31.5+x8,17.5+y8, Point.CARTESIAN), Math.toRadians(-45), Math.toRadians(-43), false, .8);
        robot.setArm(12+x5,5,true);
        robot.setTwist(0.83+x12, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setArm(12+x5,0,false);
        robot.queuer.addDelay(0.4);
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
        //drop3
        robot.followPath(new Point(22.5,22, Point.CARTESIAN), -Math.PI/4, 0, false);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB,true);
        robot.queuer.addDelay(.9);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        //grab4
        robot.followPath(new  Point(23, 36.5, Point.CARTESIAN),0,0, false,.8);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.followPath(new Point(16, 36, Point.CARTESIAN), 0, 0, false, false);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.queuer.addDelay(0.4);
    }

    public void cycleBlueGrab(int i) {
        robot.queuer.addDelay(0.2);
        robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true);
        Vector2d dist3 = new Vector2d(DIST3_X, DIST3_Y);
        robot.followPath(new Point(17 + dist3.getX(), 36 + dist3.getY(), Point.CARTESIAN), 0,0,false,false);

        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.addDelay(0.2);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
//        robot.queuer.addDelay(0.3);
    }

    public void placeSpeci2(int i) {
        robot.followPath(new Point(21, 64 + i, 1),new Point(36 + x7, 64 + i, Point.CARTESIAN), 0,0, false,0.9);
        robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, true);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
    }

    public void park() {
        robot.followPath(new Point(17, 20, 1), 0, 0, false, false);
        robot.autoReset(true);
        robot.setArm(0, 0, true);
        robot.queuer.addDelay(1);
        robot.queuer.queue(false,true);
    }

    public void update() {
        shouldPark = false;
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}