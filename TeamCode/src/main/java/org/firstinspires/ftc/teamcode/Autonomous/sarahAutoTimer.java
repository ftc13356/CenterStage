package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous()
public class sarahAutoTimer extends LinearOpMode {
    private sarahAutoUtilityTimer fatty;

    @Override
    public void runOpMode() throws InterruptedException {
        fatty = new sarahAutoUtilityTimer(this);

        waitForStart();
        if (!isStopRequested() && opModeIsActive()) {
            fatty.moveForward(10);
            fatty.turn(90);
        }
    }
}
