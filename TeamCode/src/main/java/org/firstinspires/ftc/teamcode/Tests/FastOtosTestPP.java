package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//@Disabled
@Autonomous(name = "PPLocalizerTest")
public class FastOtosTestPP extends LinearOpMode {
    Queuer queuer;
    private String navigation = "forward";

    private Follower follower;
    private PathChain trajSeq2;
    public void buildPaths() {
        //1/2.4
        //35.25 -> 14.69
        //57.75 -> 24.06
        //13.75 -> 5.73
        trajSeq2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(14.69, 24.06, (int) Math.toRadians(270)), new Point(14.69, 0, (int) Math.toRadians(180))))
                .addPath(new BezierLine(new Point(14.69, 0, (int)Math.toRadians(180)), new Point(5.73, 0, (int)Math.toRadians(90))))
                .addPath(new BezierLine(new Point(5.73, 0, (int)Math.toRadians(90)), new Point(14.69,24.06,(int)Math.toRadians(270))))
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap);
        BasicRobot robot = new BasicRobot(this, false);
        queuer = new Queuer();
        int loops = 0;

        waitForStart();

        if (isStopRequested()) return;

        buildPaths();

        resetRuntime();
        BasicRobot.time = 0;
        while(!isStopRequested()&&opModeIsActive()) {
//            for(int i=0;i<8;i++){
//                followTrajAsync(trajSeq2);
//            }
//            loops++;
//            queuer.setFirstLoop(false);
            robot.update();
            follower.update();
        }
    }
    public void followTrajAsync(PathChain traj){
        if(queuer.queue(false, !queuer.isFirstLoop()&&queuer.isExecuted()&&!follower.isBusy())){
            if(!follower.isBusy()) {
                follower.followPath(traj);
            }
        }
    }
}
