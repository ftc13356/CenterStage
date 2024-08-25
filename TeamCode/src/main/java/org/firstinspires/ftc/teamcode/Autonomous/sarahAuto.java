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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Autonomous.sarahAutoUtility;

@Autonomous()
public class sarahAuto extends LinearOpMode {
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    @Override
    public void runOpMode() throws InterruptedException {
        backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
        backRight = hardwareMap.get(DcMotor.class, "motorRightBack");

    }
}
