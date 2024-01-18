package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFColorSensor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.ArrayList;

/**
 * Harry
 * test color sensor, must have dashboard tuneable ranges for each color, print out read values, which range it is in, include LED on the extended class test program
 */
@TeleOp(name = "RFColorSensorTest")
public class RFColorSensorTest extends LinearOpMode{
    RFColorSensor colorSensor;
    public void runOpMode() throws InterruptedException{
        BasicRobot robot = new BasicRobot(this, true);
        colorSensor = new RFColorSensor("colorSensor");
        waitForStart();
        if(isStopRequested()) return;
        double count = 0;
        double total = 0;
        double avg = 0;
        while(opModeIsActive() && !isStopRequested()){
            if(BasicRobot.time > 5){
                total += colorSensor.getRawColor();
                count++;
                avg = total/count;
            }
            packet.put("Color", colorSensor.getColor());
            packet.put("%E WHITE", colorSensor.getWhite());
            packet.put("%E YELLOW", colorSensor.getYellow());
            packet.put("%E GREEN", colorSensor.getGreen());
            packet.put("%E PURPLE", colorSensor.getPurple());
            packet.put("Dist(in.)", colorSensor.getDist());
            packet.put("Correct #", count);
            packet.put("Data point #", count);
            packet.put("AVGRAWCOLOR", avg);
            robot.update();
        }
    }
}