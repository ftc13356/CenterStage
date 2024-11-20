package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CapCam;
import org.firstinspires.ftc.teamcode.Colorsens;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.Arrays;

@Autonomous
@Config
public class MOITurret extends LinearOpMode {
    public static double TARGET = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, true);
        Turret turret = new Turret();
        waitForStart();
        while (opModeIsActive()) {
            turret.goTo(TARGET);
            turret.update();
            robot.update();
        }
    }
}
