package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BlueLeft20 extends LinearOpMode {
    BL20 robot;
    public void runOpMode() throws InterruptedException {
        robot = new BL20(this);
        while(!isStopRequested()&&opModeIsActive()){
            robot.placeSamples(); //move to constructor
            robot.park();
            robot.update();
        }
    }
}