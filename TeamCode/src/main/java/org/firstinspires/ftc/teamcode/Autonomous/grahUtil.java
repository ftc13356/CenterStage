package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class grahUtil {
    private Follower follower;

    private PathChain grah;

    public static double radius = 10;
    public static double initialMovement = 45; //55
    public static double secondMovement = 35;

    public LinearOpMode op;

    public grahUtil(LinearOpMode opmode){
        op = opmode;
        follower = new Follower(op.hardwareMap);
    }

    public void followPath(){
        grah = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(initialMovement - radius,0, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement - radius,0, Point.CARTESIAN), new Point(initialMovement, 0, Point.CARTESIAN), new Point(initialMovement, -radius, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement, -radius, Point.CARTESIAN), new Point(initialMovement + secondMovement, -radius, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement + secondMovement, - radius, Point.CARTESIAN), new Point(initialMovement - secondMovement, -radius, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement, -radius, Point.CARTESIAN), new Point(initialMovement, 0, Point.CARTESIAN), new Point(initialMovement-radius, 0, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement, 0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)))
                .build();

        follower.followPath(grah);
        while(!follower.atParametricEnd() && op.opModeIsActive() && !op.isStopRequested()){
            follower.update();
        }
    }
}

/*

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));

circle = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(RADIUS,0, Point.CARTESIAN), new Point(RADIUS, RADIUS, Point.CARTESIAN)))
                    .addPath(new BezierCurve(new Point(RADIUS, RADIUS, Point.CARTESIAN), new Point(RADIUS,2*RADIUS, Point.CARTESIAN), new Point(0,2*RADIUS, Point.CARTESIAN)))
                    .addPath(new BezierCurve(new Point(0,2*RADIUS, Point.CARTESIAN), new Point(-RADIUS,2*RADIUS, Point.CARTESIAN), new Point(-RADIUS, RADIUS, Point.CARTESIAN)))
                    .addPath(new BezierCurve(new Point(-RADIUS, RADIUS, Point.CARTESIAN), new Point(-RADIUS,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)))
                    .build();

            follower.followPath(circle);
            while(!follower.atParametricEnd() && op.opModeIsActive() && !op.isStopRequested()){
                follower.update();
 */