package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class straightBackForthQueuer extends LinearOpMode {
    Queuer queuer;
    PathChain square;
    double distance = 12;

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap);
        BasicRobot robot = new BasicRobot(this, false);
        queuer = new Queuer();
        int loops = 0;

        waitForStart();
        if (isStopRequested()) return;
        PathChain trajSeq2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(distance,0,Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(distance,0, Point.CARTESIAN), new Point(0,0,Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)
                .build();

        resetRuntime();
        BasicRobot.time = 0;
        while(!isStopRequested()&&opModeIsActive()) {
            for(int i=0;i<8;i++){
                followTrajAsync(trajSeq2);
            }
            loops++;
            queuer.setFirstLoop(false);
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
