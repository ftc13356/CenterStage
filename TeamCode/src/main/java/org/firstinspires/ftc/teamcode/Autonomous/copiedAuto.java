package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class copiedAuto extends LinearOpMode {
    private copiedAuto copy;

    @Override
    public void runOpMode() throws InterruptedException {
        copy = new copiedAutoUtil(this);
        waitForStart();
        if(!isStopRequested() && opModeIsActive()){
            copy.followPath();
        }
    }
}
