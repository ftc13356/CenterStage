package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class AutoGrabTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoGrabTest aut = new AutoGrabTest(this);
        waitForStart();
        while (opModeIsActive() && !aut.robot.queuer.isFullfilled()){
            aut.autoGrab();
            aut.update();
        }
        stop();
    }
}
