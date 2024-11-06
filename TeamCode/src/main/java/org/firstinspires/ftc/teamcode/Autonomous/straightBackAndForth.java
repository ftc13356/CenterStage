package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;

@Autonomous
public class straightBackAndForth extends LinearOpMode {
    Queuer queuer;

    @Override
    public void runOpMode() throws InterruptedException {
        SBAF robot;
        robot = new SBAF(this);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            robot.hmph();
            //robot.setBackdropGoalPose();
            //robot.buildPaths();
            //robot.update(); //this might just constantly switch between forward and backward, since idk if followPath follows the path till the very end or it just sets the robot along the path
        }

    }
}
