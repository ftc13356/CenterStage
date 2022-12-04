package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LEDStrip{
    RevBlinkinLedDriver blinkin;

public LEDStrip(){
    blinkin = op.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
}



    public void rainbow(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    public void red(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void blue(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void orange(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    }

    public void yellow(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void gold(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
    }

    public void white(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void gray(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
    }

    public void pink (){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }

    public void green (){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
}

