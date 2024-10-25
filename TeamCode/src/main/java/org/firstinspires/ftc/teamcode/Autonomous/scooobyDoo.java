package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Config
@Autonomous (name = "scooobyDoo")
public class scooobyDoo extends OpMode {
    private Follower follower;
    public PathChain help;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    public static double initialMovement = 20;
    public static double radius = 20;
    public static double secondMovement = -40;

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


        help = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(initialMovement - radius, 0, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement - radius, 0, Point.CARTESIAN), new Point(initialMovement, 0, Point.CARTESIAN), new Point(initialMovement, -radius, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement, -radius, Point.CARTESIAN), new Point(initialMovement, secondMovement, Point.CARTESIAN)))

                .addPath(new BezierCurve(new Point(initialMovement, secondMovement, Point.CARTESIAN), new Point(initialMovement, -radius, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement, -radius, Point.CARTESIAN), new Point(initialMovement, 0, Point.CARTESIAN), new Point(initialMovement - radius, 0, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(initialMovement - radius, 0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)))
                .build();

        follower.followPath(help);
    }

    public void loop() {
        follower.update();
        if(follower.atParametricEnd()){
            follower.followPath(help);
        }
    }
}
