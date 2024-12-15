package org.firstinspires.ftc.teamcode.TeleOp.Comp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.IDRobot;

@TeleOp
public class RedIDTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IDRobot robot = new IDRobot(this, true);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            robot.teleOp(false);
        }
    }
}
