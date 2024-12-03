package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Config
@Autonomous(name = "ClawTest")
public class ClawTest extends LinearOpMode{
    //0open, 1closed
    public static double POS = 0;
    Claw clawServo;

    public void runOpMode() throws InterruptedException{
        BasicRobot robot = new BasicRobot(this, true);
        clawServo = new Claw();
        waitForStart();
        clawServo.goTo(POS);
        while(opModeIsActive() && !isStopRequested()){
        }
    }
}
