package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class circle extends LinearOpMode {
    private circleUtil ppLol;

    @Override
    public void runOpMode() throws InterruptedException {
        ppLol = new circleUtil(this);
        waitForStart();
        if(!isStopRequested() && opModeIsActive()){
            ppLol.followPath();
        }
    }
}
