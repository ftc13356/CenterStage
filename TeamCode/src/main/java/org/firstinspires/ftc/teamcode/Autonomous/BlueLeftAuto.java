package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BlueLeftAuto extends LinearOpMode {
    BLA robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BLA(this);
        while(!isStopRequested()&&opModeIsActive()){
            robot.placeSpeci();
            robot.placeSamples();
            robot.park();
            robot.updateFollower();
        }
    }
}