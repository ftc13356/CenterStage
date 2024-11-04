package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class straightBackAndForth extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        straightBackAndForthUtil copy;
        copy = new straightBackAndForthUtil(this);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            copy.setBackdropGoalPose();
        }
    }
}
