package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
@Disabled

@Autonomous
public class BlueRight40Sweep extends LinearOpMode {
    BR40 aut;
    public void runOpMode() throws InterruptedException {
        aut = new BR40(this);
        waitForStart();
        resetRuntime();
        BasicRobot.time=0;
        while(!isStopRequested()&&opModeIsActive()&&!aut.robot.queuer.isFullfilled()){
            aut.placeSpeci();
            aut.grabBluesSweep();
            aut.placeSpeci2(3);
            aut.cycleBlueGrab(0);
//            aut.placeSpeci2(6);
//            aut.cycleBlueGrab(0);
//            aut.placeSpeci2(9);
//            aut.park();
            aut.update();
        }
    }
}