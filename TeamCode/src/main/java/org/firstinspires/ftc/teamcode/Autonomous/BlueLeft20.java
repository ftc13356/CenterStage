package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BlueLeft20 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BL20 auto = new BL20(this);
        waitForStart();
        while (opModeIsActive() && !isStopRequested());
    }
}
