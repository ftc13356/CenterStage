package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class straightBackAndForth extends LinearOpMode {
    private straightBackAndForth copy;

    @Override
    public void runOpMode() throws InterruptedException {
        copy = new straightBackAndForthUtil(this);
        waitForStart();
        if(!isStopRequested() && opModeIsActive()){
            copy.followPath();
        }
    }
}
