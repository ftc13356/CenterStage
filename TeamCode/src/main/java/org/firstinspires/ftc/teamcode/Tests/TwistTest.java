package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
@Disabled
@Config
@Autonomous(name = "TwistTest")
public class TwistTest extends LinearOpMode{
    //0open, 1closed
    public static double POS = 0;
    BasicRobot robot;
    Twist twistServo;

    public void runOpMode() throws InterruptedException{
        robot = new BasicRobot(this, true);
        twistServo = new Twist();
        waitForStart();
        twistServo.twistTo(POS);
        while(opModeIsActive() && !isStopRequested()){
        }
    }
}
