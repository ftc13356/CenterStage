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
public class Capstone extends LinearOpMode {
    public static double G = 9.8, theta = 30, vt;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, true);
        Spinna spinna = new Spinna();
        Turret turret = new Turret();
        CapCam cam = new CapCam();
        Colorsens color = new Colorsens();
        Trigga trigger = new Trigga();
        boolean isReset = false;
        int targ = 0;
        waitForStart();
        while (opModeIsActive()){
            targ = color.getColor();
            if(targ!=0)
                cam.swapInt(targ-1);
            if(turret.isStationary()){
                if(!isReset) {
                    cam.resetCenter();
                    isReset = true;
                }
                if(!Arrays.equals(cam.getCenter(), new double[]{0, 0, 0})){
                    double[] center = cam.getCenter();
                    cam.resetCenter();
                    if(center[1]!=0){
                        double angle = atan2(center[2],center[1]);
                        turret.goTo(turret.getRot()+angle);
                        packet.put("dletaangle", angle);
                        packet.put("target", turret.getRot()+angle);
                        continue;
                    }
                    double speeed = rangeToVelo(center[2], center[0]);
                    spinna.spin(speeed);
                    if(abs(speeed-spinna.getVel())<30){
                        trigger.shoot();
                    }
                }
            } else{
                isReset = false;
            }
            trigger.update();
            turret.update();
            robot.update();
        }

    }
    public double rangeToVelo(double x, double y){
        double numrator = x * G / Math.cos(theta*PI/180);
        double power = -x*tan(theta*PI/180)*G/(vt*vt) + y*G/(vt*vt) - 1;
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

            if (abs(wPlus1 - w) < 1e-10) {
                return wPlus1;
            }

            w = wPlus1;
        }

        throw new RuntimeException("Lambert W function did not converge.");
    }
}
