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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CapCam;
import org.firstinspires.ftc.teamcode.Colorsens;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
@Config
public class Capstone extends LinearOpMode {
    public static double G = 9.8055, theta = 25, vt = 12.9, downAng = -24, WHITE_CONST = 0.11, X_OFF =14, Y_OFF = 6, A_OFF = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, true);
        Spinna spinna = new Spinna();
        Turret turret = new Turret();
        CapCam cam = new CapCam();
        double[] targetAngle = {0, 0};
        ArrayList<double[]> targetPos = new ArrayList<>();
        Colorsens color = new Colorsens();
        Trigga trigger = new Trigga();
        boolean isReset = false;
        int targ = 0;
        cam.startStreamin();
        waitForStart();
//        while(opModeIsActive()) {
//            double[] relCent = cam.getCenter().clone();
//            if (!Arrays.equals(relCent, new double[]{0, 0, 0})) {
////            targeted = true;
//                cam.resetCenter();
//                double rel = relCent[0];
//                relCent[0] = relCent[2] * Math.sin(downAng * PI / 180) + relCent[0] * Math.cos(downAng * PI / 180) + 5;
//                relCent[2] = sqrt(relCent[2] * relCent[2] + rel * rel - relCent[0] * relCent[0]);
//                double angle = Math.atan2(relCent[1], relCent[2])*180/PI;
////            if (abs(angle) < 5) {
////                targetAngle[i] = turret.getRot() + angle;
////            }
////                turret.setPower(0);
//                packet.put("relCent0", relCent[0]);
//                packet.put("relCent1", relCent[1]);
//                packet.put("relCent2", relCent[2]);
//                packet.put("TARGANGDel", angle);
//            }
//            robot.update();
//        }
        for (int i = 0; i < 2; i++) {
            if (i == 0) {
                cam.swapRed();
            } else {
                cam.swapBlue();
            }
            sleep(200);
            boolean targeted = false;
            targetPos.add(new double[]{50,-10});
            targetPos.add(new double[]{50,-10});
            for (int j = -40; j < 50; j += 15) {
                    turret.goTo(j);
                    while (opModeIsActive() && !turret.isStationaryForWHile()) {
                        turret.update();
                        robot.update();
                    }
                    double doneTIme = time;
                    cam.resetCenter();
                    while(time-doneTIme<0.2){
                        turret.update();
                        robot.update();
                    }
                    double[] relCent = cam.getCenter().clone();
                    if (!Arrays.equals(relCent, new double[]{0, 0, 0})) {
                        while(relCent[0]==0 || relCent[1]==0 || relCent[2]<20){
                            relCent = cam.getCenter().clone();
                        }
                        cam.resetCenter();
                        double rel = relCent[0];
                        relCent[0] = relCent[2] * Math.sin(downAng * PI / 180) + relCent[0] * Math.cos(downAng * PI / 180);
                        relCent[2] = sqrt(relCent[2] * relCent[2] + rel * rel - relCent[0] * relCent[0]);
                        double angle = Math.atan2(relCent[1], relCent[2])*180/PI;
                        if(abs(angle)<10) {
                            targetAngle[i] = turret.getRot() + angle+A_OFF;
                        }
                        relCent[2]+=X_OFF;
                        relCent[0]+=Y_OFF;
                        targetPos.set(i, new double[]{relCent[2], relCent[0]});
                        packet.put("relCent0", relCent[0]);
                        packet.put("relCent1", relCent[1]);
                        packet.put("relCent2", relCent[2]);
                        packet.put("TARGANG", turret.getRot() + angle);
                        packet.put("TARGANGDel", angle);
                        turret.update();
                    }
            }
        }
        while (opModeIsActive()) {
            targ = color.getColor();
            if(targ!=2 && trigger.shot()) {
                turret.goTo(targetAngle[targ]);
                double speed = rangeToVelo(targetPos.get(targ)[0] * .01, targetPos.get(targ)[1] * .01);
                if(targ==1){
                    speed+=WHITE_CONST;
                }
                spinna.spin(speed);
                if (trigger.loaded) {
                    if (spinna.spinnaAtTarget() && turret.isStationaryForWHile()) {
                        trigger.shoot();
                    }
                }
            }
            trigger.update();
            turret.update();
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

        throw new RuntimeException("Lambert W function did not converge.");
    }
}
