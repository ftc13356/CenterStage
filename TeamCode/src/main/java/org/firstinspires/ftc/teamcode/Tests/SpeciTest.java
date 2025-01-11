package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
@Config
public class SpeciTest extends LinearOpMode{
    public static double BACK = -7.9, FORWARD = 8.5, TIMES = 7;
    public void runOpMode() throws InterruptedException{
        IDRobot robot = new IDRobot(this,false);
        robot.follower.setStartingPose(new Pose(0,0,0));
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            for(int i=0;i<TIMES;i++) {
                robot.setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, false);
                robot.setClaw(Claw.ClawStates.OPEN, true);
                robot.setTwist(Twist.TwistStates.SPECIMEN, true);
                robot.setFlip(Flip.FlipStates.SPECIMEN_GRAB, true);
                robot.followPath(new Point(BACK, 0, 1), 0, 0, false);
                robot.setClaw(Claw.ClawStates.CLOSED, false);
                robot.setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN, false);
                robot.queuer.addDelay(0.4);
                robot.setTwist(Twist.TwistStates.PARALLEL, true);
                robot.queuer.addDelay(0.4);
                robot.setFlip(Flip.FlipStates.SPECIMEN, true);
                robot.followPath(new Point(FORWARD, 0, 1), 0, 0, false);
                robot.setClaw(Claw.ClawStates.OPEN, false);
                robot.autoReset(false);
            }
            robot.update();
            robot.queuer.setFirstLoop(false);
        }
    }
}
