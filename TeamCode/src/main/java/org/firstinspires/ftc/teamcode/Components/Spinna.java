package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Double.NaN;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class Spinna {
    DcMotorEx rot;
    public static double rev_per_meter = 158;
    double targSpeed = 0;
    public Spinna() {
        rot = (DcMotorEx) op.hardwareMap.dcMotor.get("spinna");
        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rot.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void spin(double speed){
        packet.put("revVelocity", rot.getVelocity());
        double newey = 2.42*speed-.038*speed*speed+.3;
        targSpeed = newey * rev_per_meter;
        rot.setVelocity(newey*rev_per_meter);
    }
    public double getVel(){
        return rot.getVelocity();
    }
    public boolean spinnaAtTarget(){
        if(abs(rot.getVelocity()-targSpeed)<11){
            return true;
        }
        return false;
    }
}
