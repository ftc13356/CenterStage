package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import android.util.Log;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.ArrayList;

@TeleOp(name = "DistanceSensorTest")
public class DistanceSensorTest extends LinearOpMode {

    private Rev2mDistanceSensor leftDist;
    private Rev2mDistanceSensor rightDist;

    public void runOpMode(){
        BasicRobot robot = new BasicRobot(this, true);
        leftDist = op.hardwareMap.get(Rev2mDistanceSensor.class, "leftDist");
        rightDist = op.hardwareMap.get(Rev2mDistanceSensor.class, "rightDist");


        waitForStart();
        while(opModeIsActive()){
            op.telemetry.addData("left distance", leftDist.getDistance(DistanceUnit.INCH));
            op.telemetry.addData("right distance", rightDist.getDistance(DistanceUnit.INCH));
            op.telemetry.update();

            robot.update();
        }
    }
}
