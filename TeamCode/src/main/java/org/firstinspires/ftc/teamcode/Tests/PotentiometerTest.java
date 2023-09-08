package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.ArrayList;

@TeleOp(name = "PotentiometerTest")
public class PotentiometerTest extends LinearOpMode {

    private AnalogInput potentiometer;
    private Servo potentiometerServo;

    public void runOpMode(){
        BasicRobot robot = new BasicRobot(this, true);
        potentiometer = op.hardwareMap.get(AnalogInput.class, "potentiometer");
        potentiometerServo = op.hardwareMap.get(Servo.class, "potentiometerServo");


        waitForStart();
        while(opModeIsActive()){
            op.telemetry.addData("voltage", potentiometer.getVoltage());
            op.telemetry.addData("angle in degrees", potentiometer.getVoltage() * 270/3.305);
            op.telemetry.addData("servo position", potentiometerServo.getPosition());
            op.telemetry.update();

            if (op.gamepad1.x) {
                potentiometerServo.setPosition(1);
            }
            if (op.gamepad1.b) {
                potentiometerServo.setPosition(0);
            }
            if (op.gamepad1.y) {
                Log.i("Servo Ending Position:", " " + potentiometer.getVoltage() * 270/3.305);
            }
            robot.update();
        }
    }
}

