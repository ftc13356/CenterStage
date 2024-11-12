package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueLeftSpeciSample extends LinearOpMode {
    private BLSS robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BLSS(this);
        waitForStart();
        while(!isStopRequested()&&opModeIsActive()){
            robot.placeSpeci();
            robot.placeSample();
            robot.update();
        }
    }
}
