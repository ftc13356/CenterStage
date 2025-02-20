package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Autonomous

public class BlueLeft04 extends LinearOpMode {
    BL04 aut;
    public void runOpMode() throws InterruptedException {
        aut = new BL04(this);
        while(!isStarted()){
            aut.initLoop();
        }
        resetRuntime();
        BasicRobot.time=0;
        while(!isStopRequested()&&opModeIsActive()&&!aut.robot.queuer.isFullfilled()&&BasicRobot.time<30.){
            aut.nonSubCycles();
            aut.autoGrahCycle(0);
            aut.autoGrahCycle(1);
            aut.autoGrahCycle(2);
            aut.autoGrahCycle(3);
            if(BasicRobot.time>28&& !aut.isDroppi()){
                break;
            }
            aut.park();
            aut.update();
        }
        aut.robot.queuer.reset();
        while(!isStopRequested()&&opModeIsActive()&&!aut.robot.queuer.isFullfilled()&&BasicRobot.time<30.){
            aut.park();
            aut.update();
        }

    }
}