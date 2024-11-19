package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Twist;

@Config
@Autonomous(name = "TwistTest")
public class TwistTest extends LinearOpMode{
    //0open, 1closed
    public static int WHICH_STATE = 0;
    Twist twistServo;

    public void runOpMode() throws InterruptedException{
        twistServo = new Twist();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            twistServo.twistTo(Twist.TwistStates.values()[WHICH_STATE]);
        }
    }
}
