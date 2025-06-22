package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BigLight;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;
@Config
@Autonomous(name = "BigLightTest")
public class BigLightTest extends LinearOpMode{
    //0open, 1closed
    public static double POS = 1;
    BigLight light;

    public void runOpMode() throws InterruptedException{
        IDRobot robot = new IDRobot(this, true);
        light = new BigLight();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
//            light.goTo(POS);
            robot.setBigLight(BigLight.BigLightStates.ON, true);
            packet.put("tar_light", POS);
        }
    }
}
