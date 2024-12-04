package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.tan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
@Config
@Autonomous
public class TestShoot extends LinearOpMode {
    public static double G = 9.8, theta = 30, vt = 14.91, TARGET_X = 0.5, TARGET_Y = -0.08, TARGET_VEL = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, true);
        Spinna spinna = new Spinna();
//        Turret turret = new Turret();
//        Colorsens color = new Colorsens();
//        Trigga trigger = new Trigga();
        int targ = 0;
        waitForStart();
        while (opModeIsActive()) {
//            targ = color.getColor();//
            double speeed = TARGET_VEL;
            packet.put("speed", speeed);
            spinna.spin(speeed);
            robot.update();
        }
    }


public double rangeToVelo(double x, double y) {
    double numrator = x * G / Math.cos(theta * PI / 180);
    double power = -x * tan(theta * PI / 180) * G / (vt * vt) + y * G / (vt * vt) - 1;
    double denom = vt * (lambertW0(-pow(Math.E, power)) + 1);
    return numrator / denom;
}

public static double lambertW0(double x) {
    if (x < -1 / Math.E) {
        throw new IllegalArgumentException("x must be >= -1/e");
    }

    double w = x; // Initial guess

    for (int i = 0; i < 100; i++) {
        double eW = Math.exp(w);
        double wEW = w * eW;
        double wPlus1 = w - (wEW - x) / (eW * (w + 1) - (w + 2) * (wEW - x) / (2 * w + 2));

        if (abs(wPlus1 - w) < 1e-10) {
            return wPlus1;
        }

        w = wPlus1;
    }

    throw new RuntimeException("Lambert W function did not converge." + x);
}
}
