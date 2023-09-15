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

@TeleOp(name = "AllSensorTest")
public class AllSensorTest extends LinearOpMode {

    private TouchSensor breakBeam;
    private AnalogInput potentiometer;
    private AnalogInput axonPotentiometer;
    private Servo clawServo;
    private Servo potentiometerServo;
    private TouchSensor magneticLimitSwitch;

    public void runOpMode(){
        BasicRobot robot = new BasicRobot(this, true);
        breakBeam = op.hardwareMap.get(TouchSensor.class, "breakBeam");
        potentiometer = op.hardwareMap.get(AnalogInput.class, "potentiometer");
        axonPotentiometer = op.hardwareMap.get(AnalogInput.class, "axonPotentiometer");
        potentiometerServo = op.hardwareMap.get(Servo.class, "potentiometerServo");
        clawServo = op.hardwareMap.get(Servo.class, "clawServo");
        magneticLimitSwitch = op.hardwareMap.get(TouchSensor.class, "magnetic");

        clawServo.setPosition(0.2);

        waitForStart();
        while(opModeIsActive()){
            op.telemetry.addData("pressed", breakBeam.isPressed());
            op.telemetry.addData("potentiometer voltage", potentiometer.getVoltage());
            op.telemetry.addData("magnetic voltage", magneticLimitSwitch.getValue());
            op.telemetry.addData("angle in degrees", (axonPotentiometer.getVoltage() - 1.51) * 142/0.59);
            op.telemetry.update();

            if (op.gamepad1.x) {
                clawServo.setPosition(0.6);
            }
            if (op.gamepad1.b) {
                clawServo.setPosition(0.2);
            }

            if (op.gamepad1.a) {
                potentiometerServo.setPosition(0.2);
            }
            if (op.gamepad1.y) {
                potentiometerServo.setPosition(0.8);
            }

            robot.update();
        }
    }
}
