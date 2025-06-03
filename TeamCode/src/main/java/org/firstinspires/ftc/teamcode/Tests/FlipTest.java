package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Components.Flip.FlipStates.SPECIMEN;
import static org.firstinspires.ftc.teamcode.Components.Flip.SPECIMEN_POS;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
//@Disabled
@Config
@Autonomous(name = "FlipTest")
public class FlipTest extends LinearOpMode{
    //0reset, 1sub, 2spec, 3specgrab, 4basket
    public static double POS = SPECIMEN_POS-.02;
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
