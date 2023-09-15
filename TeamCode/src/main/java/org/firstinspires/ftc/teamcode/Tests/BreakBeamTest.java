package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.ArrayList;

@TeleOp(name = "BreakBeamTest")
public class BreakBeamTest extends LinearOpMode {

    private TouchSensor breakBeam;

    public void runOpMode(){
        BasicRobot robot = new BasicRobot(this, true);
        breakBeam = op.hardwareMap.get(TouchSensor.class, "breakBeam");

        waitForStart();
        while(opModeIsActive()){
            op.telemetry.addData("pressed", breakBeam.isPressed());
            op.telemetry.update();

            robot.update();
        }
    }
}
