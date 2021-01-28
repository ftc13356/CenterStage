package org.firstinspires.ftc.teamcode.Qualifier_1_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1_1.Robot;

@Autonomous(name = "IMUTest")
public class IMUTest extends LinearOpMode {
    @Override

    public void runOpMode() {

        Robot robot=new Robot(this, BasicChassis.ChassisType.IMU);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        robot.moveForward(24, 0.5);
        sleep(1000);
        robot.turnInPlace(0, 0.5);
        robot.moveBackward(24, 0.5);
        sleep(1000);
        robot.turnInPlace(0, 0.5);
        robot.moveLeft(36, 0.5);
        sleep(1000);
        robot.turnInPlace(0, 0.5);
        robot.moveRight(36, 0.5);
        sleep(1000);
        robot.turnInPlace(0, 0.5);

    }
}