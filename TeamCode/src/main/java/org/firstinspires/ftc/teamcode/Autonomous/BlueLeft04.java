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
        waitForStart();
        resetRuntime();
        BasicRobot.time=0;
        while(!isStopRequested()&&opModeIsActive()&&!aut.robot.queuer.isFullfilled()){
            aut.nonSubCycles();
            aut.update();
        }
    }
}