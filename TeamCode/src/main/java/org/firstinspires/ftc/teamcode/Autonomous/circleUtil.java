package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class circleUtil {
        public static double RADIUS = 10;

        private Follower follower;

        private PathChain circle;

        public LinearOpMode op;

        public circleUtil(LinearOpMode opmode){
            op = opmode;
            follower = new Follower(op.hardwareMap);
        }

        public void followPath() {
            circle = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(RADIUS,0, Point.CARTESIAN), new Point(RADIUS, RADIUS, Point.CARTESIAN)))
                    .addPath(new BezierCurve(new Point(RADIUS, RADIUS, Point.CARTESIAN), new Point(RADIUS,2*RADIUS, Point.CARTESIAN), new Point(0,2*RADIUS, Point.CARTESIAN)))
                    .addPath(new BezierCurve(new Point(0,2*RADIUS, Point.CARTESIAN), new Point(-RADIUS,2*RADIUS, Point.CARTESIAN), new Point(-RADIUS, RADIUS, Point.CARTESIAN)))
                    .addPath(new BezierCurve(new Point(-RADIUS, RADIUS, Point.CARTESIAN), new Point(-RADIUS,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)))
                    .build();

            follower.followPath(circle);
            while(!follower.atParametricEnd() && op.opModeIsActive() && !op.isStopRequested()){
                follower.update();
            }

        }

    }

