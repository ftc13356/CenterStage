package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class straightBackAndForth extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SBAF robot;
        robot = new SBAF(this);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            robot.setBackdropGoalPose();
            robot.buildPaths();
            robot.update(); //this might just constantly switch between forward and backward, since idk if followPath follows the path till the very end or it just sets the robot along the path
        }

    }
}
