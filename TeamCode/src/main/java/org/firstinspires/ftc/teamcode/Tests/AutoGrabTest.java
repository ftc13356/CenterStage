package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
public class AutoGrabTest {
    IDRobot robot;
    public AutoGrabTest(LinearOpMode opmode) {
        robot = new IDRobot(opmode, false);
        robot.follower.setStartingPose(new Pose(57.9443447037702, 99.23339317773788, toRadians(-90)));
    }


    public void autoGrab() {
        robot.queuer.queue(false, true);
        robot.autoGrab(2);
        robot.queuer.addDelay(1);
        robot.followPathNotTargeted(new Point(80.9443447037702, 99.23339317773788,1), new Point(55.5, 99.23339317773788,1),
                0.4,toRadians(-90),toRadians(-90),true);

        robot.queuer.waitForFinish();
        robot.queuer.addDelay(1);
        robot.queuer.queue(false, true);
        robot.setArm(0, 15, false);
        robot.queuer.addDelay(0.3);
        robot.setFlip(Flip.FlipStates.RESET, true);
        robot.queuer.addDelay(0.3);
        robot.setTwist(Twist.TwistStates.PERPENDICULAR, true);
    }

    public void update() {
        robot.queuer.setFirstLoop(false);
        robot.update();
    }
}