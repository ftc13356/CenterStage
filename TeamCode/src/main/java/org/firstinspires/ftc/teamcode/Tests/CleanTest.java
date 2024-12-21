package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous
public class CleanTest extends LinearOpMode {
    public static double X = 0, Y = 8.7;
    @Override
    public void runOpMode() throws InterruptedException {
        IDRobot robot = new IDRobot(this, false);
        robot.follower.setStartingPose(new Pose(7.5,64,0));
        waitForStart();
        while(!isStopRequested()&& opModeIsActive()&&!robot.queuer.isFullfilled()){
            robot.followPath(new Point(7.5+X,64-Y,1), 0,0,false);
            robot.queuer.addDelay(3);
            robot.queuer.queue(false, BasicRobot.time>6);
            robot.update();
        }
        robot.follower.startTeleopDrive();
        robot.follower.setTeleOpMovementVectors(0,0,0);
        stop();
    }
}
