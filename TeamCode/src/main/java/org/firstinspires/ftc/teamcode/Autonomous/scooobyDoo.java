package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Config
@Autonomous (name = "scooobyDoo")
public class scooobyDoo extends OpMode {
    private Follower follower;
    public PathChain forwards, backwards;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private Telemetry telemetryA;

    public static double initialMovement = 20;
    public static double radius = 20;
    public static double secondMovement = -40;
    boolean goingForward = true;

    @Override
    public void init(){
        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a curve going " + initialMovement + " inches"
                + " to the right and" + secondMovement + " inches forward. The robot will go"
                + "forward and backward continuously along the path.");
        telemetryA.update();

        forwards = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(initialMovement - radius, 0, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement - radius, 0, Point.CARTESIAN), new Point(initialMovement, 0, Point.CARTESIAN), new Point(initialMovement, -radius, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement, -radius, Point.CARTESIAN), new Point(initialMovement, secondMovement, Point.CARTESIAN)))
                .build();

        follower.followPath(forwards);

        backwards = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialMovement, secondMovement, Point.CARTESIAN), new Point(initialMovement, -radius, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement, -radius, Point.CARTESIAN), new Point(initialMovement, 0, Point.CARTESIAN), new Point(initialMovement - radius, 0, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement - radius, 0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)))
                .build();

    }

    public void loop() {
        follower.update();
        if(follower.atParametricEnd()){
            if(goingForward){
                goingForward = false;
                follower.followPath(backwards);
            } else {
                goingForward = true;
                follower.followPath(forwards);
            }
        }
        telemetryA.addData("direction:", goingForward);
        follower.telemetryDebug(telemetryA);
    }
}
