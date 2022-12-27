package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Old.Components.Localizer.OdometryTracker;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@TeleOp
public class ConeFlipperTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo flipper = hardwareMap.servo.get("flipper");
        flipper.setPosition(1);


        while (!isStopRequested()) {
            if(gamepad1.x){
                flipper.setPosition(0.0); //cone flipping position
            }
            else{
                flipper.setPosition(1);
            }
        }
    }
}
