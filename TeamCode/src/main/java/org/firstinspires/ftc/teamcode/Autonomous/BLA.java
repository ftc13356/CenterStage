package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class BLA extends IDRobot {
    PoseUpdater poseUpdater;
    Follower follower;
    PathChain scoreSpecy, scoreSampley, sampley1, sampley2, sampley3, submersibley;
    Pose starting, current;
    double sampleDel, speciDel, sampleScoreDel;
    int sampleNum;

    public BLA(LinearOpMode opmode){
        super(opmode, false);

        op=opmode;
        follower = new Follower(op.hardwareMap);
        starting = new Pose(12,108,0);
        follower.setStartingPose(starting);
        //follower.setMaxPower(0.7);

        sampleDel = 3.0;
        sampleScoreDel = 3.0;
        speciDel = 3.0;
        sampleNum = 1;

        scoreSpecy = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(12,108,Point.CARTESIAN), new Point(36,72,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(0,0)
                .build();

        sampley1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(36,72,Point.CARTESIAN), new Point(24,72,Point.CARTESIAN), new Point(24,120,0)))
                .setLinearHeadingInterpolation(0,0)
                .build();

        sampley2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(24,130,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4, 0)
                .build();

        sampley3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(24,140, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4, 0)
                .build();

        submersibley = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20,124,Point.CARTESIAN), new Point(60,110,Point.CARTESIAN), new Point(60,96,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.PI*3/4, -Math.PI/2)
                .build();

        autoReset();
        setClaw(Claw.ClawStates.CLOSED, false);
    }
    /**
    * builds path to go from current point -> sample-scoring
     */
    public void scoreSampley () {
        current = follower.getPose();
        scoreSampley = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), new Point(20,124,Point.CARTESIAN)))
                .setLinearHeadingInterpolation(0, Math.PI*3/4)
                .build();
        queuer.addDelay(sampleScoreDel);
    }

    /**
     * follows path from starting point -> specimen-scoring
     */
    public void placeSpeci(){
        queuer.queue(false, true);
        followPathAsync(scoreSpecy);
        queuer.addDelay(speciDel);
    }

    /**
     * follows paths + sets claw and arm to pick up and places yellow samples 1,2,3
     */
    public void placeSamples(){
        switch(sampleNum){
            default:
                case 1:
                followPathAsync(sampley1);
                break;
            case 2:
                followPathAsync(sampley2);
                break;
            case 3:
                followPathAsync(sampley3);
                break;
        }
        queuer.addDelay(sampleDel);
        setClaw(Claw.ClawStates.OPEN, true);
        setArm(TelescopicArm.ArmStates.INTAKE, true);
        queuer.waitForFinish();

        scoreSampley();
        setClaw(Claw.ClawStates.CLOSED, true);
        setArm(TelescopicArm.ArmStates.HIGH_BUCKET, true);
        queuer.waitForFinish();
        if(!follower.isBusy()){
            sampleNum++;
        } //sampleNum should increase only once it drops sample but my fatass does NOT know how
    }

    /**
     * parks by submersible
     */
    public void park() {
        followPathAsync(submersibley);
    }

    public void updateFollower() {
        follower.update();
        queuer.setFirstLoop(false);

        poseUpdater.update();
        packet.put("x", poseUpdater.getPose().getX());
        packet.put("y", poseUpdater.getPose().getY());
        packet.put("heading", poseUpdater.getPose().getHeading());
        packet.put("total heading", poseUpdater.getTotalHeading());
        update();
    }

    public void followPathAsync(PathChain path){
        if(queuer.queue(false, !follower.isBusy())){
            if(!follower.isBusy()){
                follower.followPath(path);
            }
        }
    }
}