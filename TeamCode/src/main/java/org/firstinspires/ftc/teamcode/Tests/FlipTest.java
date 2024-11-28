package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Config
@Autonomous(name = "FlipTest")
public class FlipTest extends LinearOpMode{
    //0reset, 1sub, 2spec, 3specgrab, 4basket
    public static double POS = 0;
    Flip flipServo;

    public void runOpMode() throws InterruptedException{
        BasicRobot robot = new BasicRobot(this, true);
        flipServo = new Flip();

        waitForStart();
        flipServo.flipTo(POS);
        while(opModeIsActive() && !isStopRequested()){
        }
    }
}
