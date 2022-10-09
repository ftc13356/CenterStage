package org.firstinspires.ftc.teamcode.Components;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



public class LedStrip extends OpMode{
    RevBlinkinLedDriver lights;

    @Override
    public void init(){
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

    }

    @Override
    public void loop() {
        //todo
    }
}
