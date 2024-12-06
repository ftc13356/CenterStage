package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import static java.lang.Double.NaN;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Config
public class Turret {
    DcMotorEx rot;
    public static double  ROTMAX = 100, ROTMIN = -100, TICKS_PER_DEG = .3617, rP = 0.02, rD = 0;
    boolean stationary;
    double TICKS_PER_RAD = TICKS_PER_DEG*PI/180, rotTarget = 0, STALL_TORQUE = 24.3;
    double stationTime = -100, curRot = 0 , curVel = 0;
    private BufferedWriter writer;
    private File logFile;
    public Turret() {
        rot = (DcMotorEx) op.hardwareMap.dcMotor.get("turt");
        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rot.setDirection(DcMotorSimple.Direction.REVERSE);
        logFile = new File("/storage/emulated/0/tmp/encoderLog.csv");

        try {
            if (logFile.createNewFile()) {
           }
            writer = new BufferedWriter(new FileWriter(logFile, true)); // Append mode
        } catch (IOException e) {
            packet.put("Error", "Failed to open log file");
            dashboard.sendTelemetryPacket(packet);
            return;
        }

        stationary=false;
        stationTime=-100;
    }

    public void goTo(double rotation){
        rotation = min(max(rotation,ROTMIN),ROTMAX);
        rotTarget = rotation;
        curRot= rot.getCurrentPosition()*TICKS_PER_DEG;
        curVel = rot.getVelocity()*TICKS_PER_DEG;
        double pos = curRot;
        double vel = curVel;
        double rErr = rotation - pos;
        double rd = -vel;
        double power = (rP)*rErr+.001*(rD)*rd;
        if(signum(vel) == signum(rErr)){
            power *= 1/(1-abs(vel/(312*537.7)));
        }
        double curTorque = STALL_TORQUE * power * (1-abs(vel/(312*537.7)));
        try {
            writer.write("Time: " + time + "s, Position: " + pos*PI/180 + ", Velocity: " + vel*PI/180 + ", Current Torque: " + curTorque);
            writer.newLine(); // Move to the next line
            writer.flush(); // Ensure data is written to file immediately
        } catch (IOException e) {
            packet.put("Error", "Failed to write data to log file");
            dashboard.sendTelemetryPacket(packet);
        }
        if(abs(power)>0.5){
            power *= abs(0.4/power);
        }
        rot.setPower(power);
        packet.put("powa",power);
        packet.put("pos", pos);
        packet.put("velo", vel);
        packet.put("curTorque", curTorque);
    }
    public boolean isStationary(){
        return abs(curVel)<5 && abs(rotTarget - curRot)< 10;
    }
    public boolean isStationaryForWHile(){
        return time-stationTime > .3 && stationTime!=-100 && abs(rotTarget - curRot)< 10;
    }
    public void setPower(double power){
        rot.setPower(power);
    }
    public double getRot(){
        return curRot;
    }

    public void update(){
        goTo((rotTarget));
        stationary = isStationary();
        if(!stationary){
            stationTime = time;
        }
        packet.put("rotTarget", rotTarget);
        packet.put("stationTime", stationTime);
        packet.put("time", time);
        packet.put("curVel", curVel);
        packet.put("stationary", stationary);
        ;
//        packet.put("vel", rot.getVelocity());
//        packet.put("pos", rot.getCurrentPosition());
    }
}
