package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@TeleOp(name = "IRTest")
public class IRTest extends LinearOpMode {

    private ModernRoboticsI2cIrSeekerSensorV3 seeker;

    public void runOpMode(){
        BasicRobot robot = new BasicRobot(this, true);
        seeker = op.hardwareMap.get(ModernRoboticsI2cIrSeekerSensorV3.class, "seeker");

        waitForStart();
        while(opModeIsActive()){
            op.telemetry.addData("angle", seeker.getAngle());
            op.telemetry.addData("distance", seeker.getStrength());
            op.telemetry.update();

            robot.update();
        }
    }
}

