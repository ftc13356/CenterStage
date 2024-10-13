package org.firstinspires.ftc.teamcode.SampleDetect;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Autonomous
public class SampleDetectTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        CameraInit cam = new CameraInit(false, this);
        cam.startStreamin();
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){

        }
        stop();
    }
}
