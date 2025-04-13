package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Autonomous
//@Disabled
public class BlueRight40 extends LinearOpMode {
    BR40 aut;
    public void runOpMode() throws InterruptedException {
        aut = new BR40(this);
        while(!isStarted()){
            aut.initLoop();
        }
        resetRuntime();
        BasicRobot.time=0;
        while(!isStopRequested()&&opModeIsActive()&&!aut.robot.queuer.isFullfilled()){
            aut.placeSpeci();
            aut.autoGrab();
            aut.grabBluesSweep();
            aut.placeSpeci2(0);
            aut.cycleBlueGrab(0);
            aut.placeSpeci2(2);
            aut.cycleBlueGrab(0);
            aut.placeSpeci2(4);
            aut.cycleBlueGrab(0);
            aut.placeSpeci2(6);
            aut.cycleBlueGrab(0);
            aut.placeSpeci2(8);
            aut.park();
            aut.update();
        }
    }
}