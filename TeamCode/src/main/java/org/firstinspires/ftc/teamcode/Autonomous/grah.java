package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class grah extends LinearOpMode {
    private grahUtil grah;
    @Override
    public void runOpMode() throws InterruptedException {
        grah = new grahUtil(this);
        waitForStart();
        if(!isStopRequested() && opModeIsActive()){
            grah.followPath();
        }
    }
}