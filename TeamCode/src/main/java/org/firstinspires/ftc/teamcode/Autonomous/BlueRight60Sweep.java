package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Autonomous
public class BlueRight60Sweep extends LinearOpMode {
    BR40 aut;
    public void runOpMode() throws InterruptedException {
        aut = new BR40(this);
        while(!isStarted()){
            aut.initLoop();
        }
        resetRuntime();
        BasicRobot.time=0;
        while(!isStopRequested()&&opModeIsActive()&&!aut.robot.queuer.isFullfilled()&&BasicRobot.time<30.0){
            aut.placeSpeci();
            aut.grabBluesSweep();
            aut.placeSpeci2(-2);
            aut.autoGrab();
            aut.autoBlueGrab(0);
            aut.placeSpeci2(3);
            aut.cycleBlueGrab(0);
            aut.placeSpeci2(7);
            aut.cycleBlueGrab(1);
            aut.placeSpeci2(10);
            aut.cycleBlueGrab(2);
            aut.placeSpeci2(13);
            aut.park();
            aut.update();
        }
    }
}

