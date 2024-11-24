package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robots.IDRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class ExampleAuto {
    PathChain path;
    IDRobot robot;
    double constant1, constant2, constant3;
    Pose poses;
    public ExampleAuto(LinearOpMode opMode){
        robot =  new IDRobot(opMode,false);
        //u can use op for opMode, i just use this line to demonstrate u don't want to make one of these
        HardwareMap map = op.hardwareMap;

//        robot.follower
    }
}
