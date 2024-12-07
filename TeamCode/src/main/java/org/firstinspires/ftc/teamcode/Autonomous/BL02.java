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
    boolean shouldPark = false;
    Pose current;

    public BL02(LinearOpMode opmode){
        robot = new IDRobot(opmode,false);
        robot.follower.setStartingPose(new Pose(7.5,103,0));
    }

    public void placeSample(){
        robot.followPath(new Point(17+x,126+y,Point.CARTESIAN), 0, -PI/4,false);
        robot.queuer.addDelay(0.7);
        robot.setArm(TelescopicArm.ArmStates.HIGH_BUCKET, false);
        robot.setFlip(Flip.FlipStates.RESET, true);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
        robot.setFlip(Flip.FlipStates.BUCKET, false);
        robot.queuer.addDelay(0.5);
        robot.setClaw(Claw.ClawStates.OPEN, false);
        robot.queuer.addDelay(1.0);
        robot.setFlip(Flip.FlipStates.RESET, false);

        current = robot.follower.getPose();
        robot.queuer.addDelay(1.0);
        robot.autoReset(false);
        robot.followPath(new Point(current.getX()+2, current.getY()-2, Point.CARTESIAN), -PI/4, -PI/4, true); //change to (18,124) once you're fine with it

        if(!shouldPark){
            robot.followPath(new Point(24,121,Point.CARTESIAN), -PI/4, 0, false);
            grabYellow();
        } else {
            park();
        }
    }

    public void grabYellow(){
        double height=4, length=19;
        double ext = length-7, rot = 180/PI *Math.atan2(height, length);
        robot.queuer.addDelay(1.0);
        robot.setArm(ext, 0, true);
        //robot.setArm(ext, rot, true);
        robot.queuer.addDelay(1.0);
        robot.setTwist(Twist.TwistStates.PARALLEL, true);
        robot.setFlip(Flip.FlipStates.SUBMERSIBLE, true);
        robot.setClaw(Claw.ClawStates.CLOSED, false);
        shouldPark = true;
        placeSample();
    }
    /**
    parks by submersible
     */
    public void park() {
        robot.followPath(new Point(60,110,Point.CARTESIAN), new Point(60,96, Point.CARTESIAN),-Math.PI/4, -Math.PI/2, false);
    }

    public void update() {
        shouldPark= false;
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}