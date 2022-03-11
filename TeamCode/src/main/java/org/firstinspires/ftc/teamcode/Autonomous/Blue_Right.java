package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "Blue_Right_Regional", preselectTeleOp = "OneGPTeleop")
public class Blue_Right extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, false, false);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        double cycletime = 15;

        robot.setPosition(0,2,0);
        waitForStart();

        robot.TurretSlidesToPosition(0, 21
                , 0, 0.5);
        sleep(1300);
        robot.FlipBasketArmToPosition(.6);
        sleep(500);
        robot.FlipBasketToPosition(1.0);
        sleep(5000);
        robot.TurretSlidesToPosition(0, 0
                , 0, 0.5);
        sleep(5000);
        stop();
    }
}