package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled

@Autonomous
public class BlueLeftSpeciSampleTest extends LinearOpMode {
    private BLSSTest robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BLSSTest(this);
        waitForStart();
        while(!isStopRequested()&&opModeIsActive()){
            robot.placeSpeci();
            robot.placeSample();
            robot.updateFollower();
        }
    }
}
