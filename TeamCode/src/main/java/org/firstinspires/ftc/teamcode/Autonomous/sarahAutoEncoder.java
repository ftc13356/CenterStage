package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous()
public class sarahAutoEncoder extends LinearOpMode {
    private sarahAutoUtilityEncoder fatty;

    @Override
    public void runOpMode() throws InterruptedException {
        fatty = new sarahAutoUtilityEncoder(this);

        waitForStart();
        if (!isStopRequested() && opModeIsActive()) {
            fatty.moveForward(10);
            fatty.turn(90);
        }
    }
}
