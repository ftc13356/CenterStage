package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous
public class straightBackAndForthTest extends LinearOpMode {
    private SBAFTest robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SBAFTest(this);
        waitForStart();
        robot.setBackdropGoalPose();
        robot.buildPaths();
        while(!isStopRequested() && opModeIsActive()){
            robot.updateFollower();
        }
    }
}