package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.ArrayList;

@TeleOp(name = "PotentiometerTest")
public class PotentiometerTest extends LinearOpMode {

    private AnalogInput potentiometer;

    public void runOpMode(){
        double lastSwitchTime = 0;
        BasicRobot robot = new BasicRobot(this, true);
        potentiometer = op.hardwareMap.get(AnalogInput.class, "potentiometer");
        waitForStart();
        while(opModeIsActive()){
            op.telemetry.addData("voltage", potentiometer.getVoltage());
            robot.updateTime();
        }
    }
}
