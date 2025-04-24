package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST3_X;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST3_Y;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.BACK_EXTEND_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.BACK_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_EXTEND_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.SPECIMENGRAB_EXTEND_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.SPECIMENGRAB_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.Hardstop;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
public class BR40 {
    IDRobot robot;
    public static double x1 = 0, x2 = 0, x3 = 0, x4 = 0, x5 = 0, x6 = 0, x7 = 0, X_OFF = -.14, Y_OFF = 0;
    public static double x8 = 0, x9 = 0, x10 = 0, x12 = 0, y9 = 0;
    public double y2 = 0, y3 = 0, y4 = 0, y5 = 0, y6 = 0, y7 = 0, y8 = 0, y10 = 0, DROP_DELAY = 0.5;
    int position = 0;
    boolean shouldPark = false, shouldAuto = true;
    int isRed = 0;
    Vector2d dist3 = new Vector2d(DIST3_X, DIST3_Y);


    public BR40(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(7.5, 64, 0));
        robot.update();
    }

    public void initLoop() {
        boolean isRD = gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "auto grab red");
        boolean isLD = gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "auto grab red");
        boolean isUD = gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "auto grab red");
        boolean isDD = gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "isRed");
//        robot.follower.setStartingPose(new Pose(7.5, 64, 0));
        if (isLD) {
            position += 1;
        }
        if (isRD) {
            position -= 1;
        }
        if (isUD) {
            shouldAuto = !shouldAuto;
        }
        if (isDD) {
            if(isRed==0)
                isRed=1;
            else
                isRed = 0;
        }
        packet.put("position", position);
        packet.put("shouldGrah", shouldAuto);
        packet.put("isred", isRed);
        op.telemetry.addData("position", position);
        op.telemetry.addData("shouldGrah", shouldAuto);
        op.telemetry.addData("isRed", isRed);
        robot.arm.goTo(0, 44);
        if(robot.arm.getRot()>39){
            robot.hardstop.goTo(Hardstop.HardstopStates.STOP);
        }
        robot.update();
        op.telemetry.update();
    }

    public void placeSpeci() {
//        robot.queuer.addDelay(0.4);
        robot.followPath(new Point(43.2 + x1, 64, Point.CARTESIAN), 0, 0, false, .85);
        robot.setArm(HIGHSPECIMEN_EXTEND_POS,HIGHSPECIMEN_PITCH_POS, true);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        robot.followPath(new Point(34.4 + x1, 64, Point.CARTESIAN), 0, 0, false, .8);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setHardstop(Hardstop.HardstopStates.GO, true);
        robot.setArm(0, 95, true);
        robot.queuer.addDelay(0.3);
        robot.setArm(0, 15, true);


    }

    public void autoGrab() {
        if (shouldAuto) {

//            robot.autoReset(false);
//            robot.followPath(new Point(36.4 + x1, 67, Point.CARTESIAN), 0, 0, false, .9);
            robot.followPath(new Point(36 + x1, 72 + position * 4, Point.CARTESIAN), 0, 0, false, .9);
            if (isRed > 0)
                robot.autoGrab(0);
            else
                robot.autoGrab(1);
            robot.queuer.waitForFinish();
            robot.queuer.addDelay(0.2);
            robot.queuer.queue(false, true);
            robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        } else {
            cycleBlueGrab(2);
        }

    }

    public void grabBluesSweep() {
        //grab1
        robot.queuer.addDelay(0.0);
        robot.followPath(new Point(26.0, 50.5, Point.CARTESIAN), -0.1 * Math.PI / 4, -3.4 * Math.PI / 4, false, 0.85);
        robot.setArm(0, 15, true);
        robot.queuer.addDelay(0.05);
        robot.setFlip(Flip.FlipStates.RESET, true);
        robot.queuer.addDelay(0.05);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(0.8);
        robot.setArm(20, 5, true);
        robot.queuer.addDelay(1.2);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.followPath(new Point(26.25 + x3, 38.5 + y3, Point.CARTESIAN), Math.toRadians(-40), Math.toRadians(-40), false, .8);
        robot.setArm(17 + x1, 5, true);
//        robot.queuer.addDelay(0.2);
        robot.setTwist(0.83 + x12, true);
        robot.queuer.addDelay(0.4);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.addDelay(0.2);
        robot.setArm(18 + x1, 0, false);
        robot.queuer.addDelay(0.1);
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
        //drop1
        robot.followPath(new Point(26.0, 34.5, Point.CARTESIAN), -3* Math.PI / 4, -3 * Math.PI / 4, false, 0.85);
        robot.setArm(17, 5, true);
        robot.queuer.addDelay(DROP_DELAY);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        //grab2
        robot.followPath(new Point(26.75 + x4, 30.0 + y4, Point.CARTESIAN), Math.toRadians(-42), Math.toRadians(-42), false, .8);
        robot.setTwist(0.83 + x12, true);
        robot.setArm(17, 4, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
//        robot.queuer.addDelay(0.2);
        robot.setArm(17, 0, false);
        robot.queuer.addDelay(0.1);
        robot.queuer.queue(false, true);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
        //drop2
        robot.followPath(new Point(26 + x7, 24 + y7, Point.CARTESIAN), -3 * Math.PI / 4, -3 * Math.PI / 4, false, 0.9);
        robot.setArm(17, 5, true);
        robot.queuer.addDelay(DROP_DELAY);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        //grab3
        robot.followPath(new Point(29.75 + x8, 16 + y8, Point.CARTESIAN), Math.toRadians(-44), Math.toRadians(-44), false, .8);
        robot.setArm(12.5 + x5, 5, true);
        robot.setTwist(0.83 + x12, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.queuer.addDelay(0.1);
        robot.setArm(12.5 + x5, 0, false);
        robot.queuer.addDelay(0.1);
        robot.queuer.queue(false, true);
//        robot.queuer.addDelay(0.2);
        robot.setClaw(Claw.ClawStates.CLOSED, true);
        //drop3
        robot.followPath(new Point(22, 24.75, Point.CARTESIAN), Math.toRadians(-44), 0, false);
        robot.setArm(SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);
        //grab4
        robot.followPath(new Point(17.5, 24.25, Point.CARTESIAN), 0, 0, false);
        robot.setHardstop(Hardstop.HardstopStates.STOP, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
//        robot.queuer.addDelay(0.4);
    }

    public void cycleBlueGrab(int i) {

        robot.followPath(new Point(24 + i * X_OFF,40.4 + i * Y_OFF,1), new Point(17.4 + i * X_OFF, 35.6 + i * Y_OFF, Point.CARTESIAN), 0, 0, false, 1, false);
        robot.setArm(0, 65, true);
        robot.queuer.addDelay(0.25);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);
        robot.queuer.addDelay(0.7);
        robot.setArm(SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS, true);
        robot.queuer.addDelay(0.2);
        robot.setTwist(Twist.TwistStates.SPECIMEN, true);
        robot.queuer.addDelay(0.2);
        robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
        robot.setClaw(Claw.ClawStates.GIGA_OPEN, true);

        robot.setClaw(Claw.ClawStates.CLOSED, false);
//        robot.queuer.addDelay(0.3);
    }




    public void placeSpeci2(int i) {
        robot.followPath(new Point(20, 66 + i, 1), new Point(39, 68 + i , Point.CARTESIAN), 0, 0, false, .85);
        robot.setArm(0, HIGHSPECIMEN_PITCH_POS, true);
        robot.queuer.addDelay(0.8);
        robot.setArm(HIGHSPECIMEN_EXTEND_POS, HIGHSPECIMEN_PITCH_POS, true);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SPECIMEN, true);

    }

    public void park() {
        robot.followPath(new Point(17, 20, 1), 0, 0, false, false);
        robot.queuer.addDelay(0.2);
        robot.setArm(0, HIGHSPECIMEN_PITCH_POS+10, true);
        robot.queuer.addDelay(1.2);
        robot.setArm(0, 0, true);;;
        robot.queuer.addDelay(1);
        robot.queuer.queue(false, true);
    }

    public void update() {
        shouldPark = false;
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}