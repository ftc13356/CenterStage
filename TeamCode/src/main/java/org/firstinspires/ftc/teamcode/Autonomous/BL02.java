package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
public class BL02 {
    IDRobot robot;
    public static double x=0;
    public static double y=0;
    public static double extra=0;
    boolean shouldPark = false;
    Pose current;

    public BL02(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);
        robot.follower.setStartingPose(new Pose(7.5,104,0));
    }

    public void placeSample(){
        robot.followPath(new Point(17.1,126.75,Point.CARTESIAN), 0, -PI/4,false);
        robot.queuer.addDelay(0.5);
        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, false);
        robot.setFlip(Flip.FlipStates.RESET, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.queuer.addDelay(0.3);
        robot.setFlip(Flip.FlipStates.BUCKET, false);
        robot.queuer.addDelay(0.5);
        robot.setClaw(Claw.ClawStates.OPEN, false);
        robot.queuer.addDelay(1.0);
        robot.setFlip(Flip.FlipStates.RESET, false);

        current = robot.follower.getPose();
        robot.queuer.addDelay(1.0);
        robot.autoReset(false);
        robot.setClaw(Claw.ClawStates.OPEN, true);
        robot.followPath(new Point(current.getX()+1, current.getY()-1, Point.CARTESIAN), -PI/4, -PI/4, true);

        if(!shouldPark){
            grabYellow();
        } else {
            park();
        }
    }

    public void grabYellow(){
        robot.followPath(new Point(27.5+x,122+y,Point.CARTESIAN), -PI/4, 0, false);
        double height=4, length=17.5+extra;
        double ext = length-7, rot = 180/PI *Math.atan2(height, length);
        robot.queuer.addDelay(1.0);
        robot.setArm(ext, 0, true);
        //robot.setArm(ext, rot, true);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        robot.queuer.waitForFinish();
        shouldPark = true;
        robot.setFlip(Flip.FlipStates.RESET, true);
        robot.setArm(TelescopicArm.ArmStates.RETRACTED, true);
        robot.queuer.waitForFinish();
        placeSample();
    }
    /**
     parks by submersible
     */
    public void park() {
        robot.followPath(new Point(65,110,Point.CARTESIAN), new Point(60,101, Point.CARTESIAN),-Math.PI/4, -Math.PI/2, false);
    }

    public void update() {
        shouldPark= false;
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}