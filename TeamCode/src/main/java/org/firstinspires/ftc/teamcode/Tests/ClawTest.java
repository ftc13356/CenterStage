package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.FutureComponents.Claw;

@Config
@Autonomous(name = "ClawTest")
public class ClawTest extends LinearOpMode{
    //0open, 1closed
    public static int WHICH_STATE = 0;
    Claw clawServo;

    public void runOpMode() throws InterruptedException{
        clawServo = new Claw();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            clawServo.goTo(Claw.ClawStates.values()[WHICH_STATE]);
        }
    }
}
