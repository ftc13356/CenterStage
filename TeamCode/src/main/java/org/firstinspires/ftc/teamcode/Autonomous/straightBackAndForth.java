package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class straightBackAndForth extends LinearOpMode {
    private SBAF robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SBAF(this);
        waitForStart();
        robot.setBackdropGoalPose();
        robot.buildPaths();
        while(!isStopRequested() && opModeIsActive()){
            robot.updateFollower();
        }

    }
}
