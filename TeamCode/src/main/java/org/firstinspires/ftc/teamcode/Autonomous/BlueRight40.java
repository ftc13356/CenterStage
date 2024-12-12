package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueRight40 extends LinearOpMode {
    BR40 aut;
    public void runOpMode() throws InterruptedException {
        aut = new BR40(this);
        waitForStart();
        while(!isStopRequested()&&opModeIsActive()){
            aut.placeSpeci();
            aut.grabBlues();
            aut.placeSpeci2(4);
            aut.cycleBlueGrab();
            aut.placeSpeci2(7);
            aut.update();
        }
    }
}