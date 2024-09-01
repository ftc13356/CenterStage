/*
-MotorTest.java
    -uses composition to access two other utility classes that allows them to use the robot functions, drive functions, and facilitates the packet functions.

- move forward 10 inches and turn 90 clockwise
    -You will have to make your own utility class. It should contain two functions, moveForward(double inches) and turn(double degrees)
    -time based
*/

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous()
public class sarahAuto extends LinearOpMode {
    private sarahAutoUtility fatty;

    @Override
    public void runOpMode() throws InterruptedException {
        fatty = new sarahAutoUtility(hardwareMap);

        waitForStart();
        if (!isStopRequested() && opModeIsActive()) {
            fatty.moveForward(10);
            fatty.turn(90);
        }
    }
}