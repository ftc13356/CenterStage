package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous
public class BlueLeft02 extends LinearOpMode {
    BL02 robot;
    public void runOpMode() throws InterruptedException {
        robot = new BL02(this);
        waitForStart();
        while(!isStopRequested()&&opModeIsActive()){
            robot.placeSample();
            robot.update();
        }
    }
}