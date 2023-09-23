package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@TeleOp(name = "RFLoggerTest")
public class RFLoggerTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        BasicRobot robot = new BasicRobot(this, true);
        boolean isPressed = false;

        waitForStart();

        while (opModeIsActive()&&!isStopRequested()) {
            if(gamepad1.x && !isPressed){
                factorial(10);
                isPressed = true;
            }
            else if(!gamepad1.x && isPressed){
                isPressed = false;
            }
//            logger2.log("Hello Warren");
            robot.update();
        }
        stop();
    }
//
    public int factorial(int n){
        if(n==0||n==1){
            int b = 1;
            logger2.logMAX("" + b);
            return b;
        }
        int a = n*factorial(n-1);
        logger2.logMAX("" + a);
        return a;
    }
}
