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
    private Servo clawServo;

    public void runOpMode(){
        BasicRobot robot = new BasicRobot(this, true);
        potentiometer = op.hardwareMap.get(AnalogInput.class, "axonPotentiometer");
        clawServo = op.hardwareMap.get(Servo.class, "clawServo");

        clawServo.setPosition(0);

        waitForStart();
        while(opModeIsActive()){
            op.telemetry.addData("voltage", potentiometer.getVoltage());
            op.telemetry.addData("angle in degrees", (potentiometer.getVoltage() - 1.51) * 142/0.59);
            op.telemetry.addData("servo position", clawServo.getPosition());
            op.telemetry.update();

            if (op.gamepad1.x) {
                clawServo.setPosition(0.6);
            }
            if (op.gamepad1.b) {
                clawServo.setPosition(0.2);
            }
            if (op.gamepad1.y) {
                Log.i("angle", "" + (potentiometer.getVoltage() - 1.51) * 142/0.59);
//                Log.i("voltage", "" + potentiometer.getVoltage());
            }
            robot.update();
        }
    }
}
