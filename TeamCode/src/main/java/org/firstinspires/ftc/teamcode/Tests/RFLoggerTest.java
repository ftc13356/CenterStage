package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Autonomous(name = "RFLoggerTest")
public class RFLoggerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);

        waitForStart();

        while (opModeIsActive()) {
            logger2.log(RFLogger.Severity.INFO, RFLogger.Files.GENERAL_LOG, "Hello World");
            robot.update();
        }

        if (isStopRequested()) return;
    }
}
