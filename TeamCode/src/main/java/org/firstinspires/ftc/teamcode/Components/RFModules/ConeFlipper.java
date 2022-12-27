package org.firstinspires.ftc.teamcode.Components.RFModules;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.hardware.Servo;

public class ConeFlipper {
    Servo flipper;
    private double downPosition = 0.0;
    private double upPosition = 1.0;
    public ConeFlipper(){
        flipper = op.hardwareMap.servo.get("flipper");
        flipper.setPosition(upPosition);
    }

    public void setDownPosition(){
        flipper.setPosition(downPosition);
    }
    public void setUpPosition(){
        flipper.setPosition(upPosition);
    }
}
