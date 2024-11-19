package org.firstinspires.ftc.teamcode.Components.SampleDetect;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Autonomous
public class SampleDetectTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        CVMaster cam = new CVMaster();
        cam.startStreamin();
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){

        }
        stop();
    }
}
