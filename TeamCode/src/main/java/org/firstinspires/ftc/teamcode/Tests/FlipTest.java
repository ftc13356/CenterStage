package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.FutureComponents.Flip;

@Config
@Autonomous(name = "FlipTest")
public class FlipTest extends LinearOpMode{
    //0reset, 1sub, 2spec, 3specgrab, 4basket
    public static int WHICH_STATE = 0;
    Flip flipServo;

    public void runOpMode() throws InterruptedException{
        flipServo = new Flip();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            flipServo.flipTo(Flip.FlipStates.values()[WHICH_STATE]);
        }
    }
}
