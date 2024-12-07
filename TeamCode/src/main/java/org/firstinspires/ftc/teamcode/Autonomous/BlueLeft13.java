//ignore this + BL31

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BlueLeft13 extends LinearOpMode {
    BL13 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BL13(this);
        while(!isStopRequested()&&opModeIsActive()){
            robot.placeSpeci();
            robot.update();
        }
    }
}