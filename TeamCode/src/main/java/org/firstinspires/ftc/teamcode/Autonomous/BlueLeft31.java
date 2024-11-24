//ignore this + BL31

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BlueLeft31 extends LinearOpMode {
    BL31 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BL31(this);
        while(!isStopRequested()&&opModeIsActive()){
            robot.placeSpeci();
            robot.placeSamples();
            robot.park();
            robot.updateFollower();
        }
    }
}