package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import android.text.method.Touch;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@TeleOp(name = "MagneticSwitchTest")
public class MagneticSwitchTest extends LinearOpMode {

    private TouchSensor magneticLimitSwitch;

    public void runOpMode(){
        BasicRobot robot = new BasicRobot(this, true);
        magneticLimitSwitch = op.hardwareMap.get(TouchSensor.class, "magnetic");

        waitForStart();
        while(opModeIsActive()){
            op.telemetry.addData("voltage", magneticLimitSwitch.getValue());
            op.telemetry.update();

            robot.update();
        }
    }
}

