package org.firstinspires.ftc.teamcode.TeleOp.Comp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.BigLight;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;

@TeleOp
public class BlueIDTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IDRobot robot = new IDRobot(this, true);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
//            robot.setBigLight(BigLight.BigLightStates.ON, true);
            robot.teleOp(true);
        }
    }
}
