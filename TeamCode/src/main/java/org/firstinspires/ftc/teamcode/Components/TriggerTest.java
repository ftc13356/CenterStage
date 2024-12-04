package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
@Config
@Autonomous
public class TriggerTest extends LinearOpMode {
    Trigga trigger;
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        trigger = new Trigga();
        waitForStart();
        trigger.shoot();
        sleep(1000);
    }
}
