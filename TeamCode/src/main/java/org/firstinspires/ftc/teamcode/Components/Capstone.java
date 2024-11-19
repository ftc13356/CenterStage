package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.pow;
import static java.lang.Math.tan;

import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
@Autonomous
@Config
public class Capstone extends LinearOpMode {
    public static double G, theta, vt;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, true);
        Spinna spinna = new Spinna();
        Turret turret = new Turret();

    }
    public double rangeToVelo(double x, double y){
        double numrator = x * G / Math.cos(theta);
        double power = -x*tan(theta)*G/(vt*vt) + y*G/(vt*vt) - 1;
        double denom = vt * (lambertW0(pow(-Math.E, power))+1);
        return numrator/denom;
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

            if (Math.abs(wPlus1 - w) < 1e-10) {
                return wPlus1;
            }

            w = wPlus1;
        }

        throw new RuntimeException("Lambert W function did not converge.");
    }
}
