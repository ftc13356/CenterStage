package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Trigga {
    Servo trigga;
    public static double LOAD=0, SHOOT=1;
    boolean loaded = true;
    double lastShootTIme = -100;
    public Trigga(){
        trigga = op.hardwareMap.get(Servo.class, "trigger");
        trigga.setPosition(LOAD);
        lastShootTIme = -100;
        loaded = true;
    }
    public void shoot(){
        trigga.setPosition(SHOOT);
        lastShootTIme = time;
        loaded = false;
    }
    public void load(){
        trigga.setPosition(LOAD);
        loaded = true;
    }

    public void update(){
        if(!loaded && time-lastShootTIme>.5){
            load();
        }
    }

}
