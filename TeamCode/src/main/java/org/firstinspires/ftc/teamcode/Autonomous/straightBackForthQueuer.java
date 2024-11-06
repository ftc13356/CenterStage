package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class straightBackForthQueuer extends LinearOpMode {
    Queuer queuer;
    PathChain square;
    double distance = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        queuer = new Queuer();
        int loops = 0;

        waitForStart();
        if(isStopRequested()) return;
        square = new PathChain(new BezierCurve(
                new Point(0,0, Point.CARTESIAN),
                new Point(-21.5, 0, Point.CARTESIAN),
                new Point(-13.75, 57.75, Point.CARTESIAN), new Point(0, 57.75, Point.CARTESIAN)));
    }
}
