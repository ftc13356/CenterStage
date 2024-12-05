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
    public static double  ROTMAX = 100, ROTMIN = -100, TICKS_PER_DEG = .3617, rP = 0.02, rD = -.4;
    boolean mid;
    double TICKS_PER_RAD = TICKS_PER_DEG*PI/180, rotTarget = 0, STALL_TORQUE = 24.3;
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

        mid=true;
    }

    public void goTo(double rotation){
        rotation = min(max(rotation,ROTMIN),ROTMAX);
        rotTarget = rotation;
        double pos = rot.getCurrentPosition();
        double vel = rot.getVelocity();
        double rErr = rotation - pos*TICKS_PER_DEG;
        double rd = -vel*TICKS_PER_DEG;
        double power = (rP)*rErr+.001*(rD)*rd;
        if(signum(vel) == signum(rErr)){
            power *= 1/(1-abs(vel/(312*537.7)));
        }
        double curTorque = STALL_TORQUE * power * (1-abs(vel/(312*537.7)));
        try {
            writer.write("Time: " + time + "s, Position: " + pos*TICKS_PER_DEG*PI/180 + ", Velocity: " + vel*TICKS_PER_DEG*PI/180 + ", Current Torque: " + curTorque);
            writer.newLine(); // Move to the next line
            writer.flush(); // Ensure data is written to file immediately
        } catch (IOException e) {
            packet.put("Error", "Failed to write data to log file");
            dashboard.sendTelemetryPacket(packet);
        }
        rot.setPower(power);
        packet.put("powa",power);
        packet.put("pos", pos*TICKS_PER_DEG);
        packet.put("velo", vel*TICKS_PER_DEG);
        packet.put("curTorque", curTorque);
    }
    public boolean isStationary(){
        return abs(rot.getVelocity())<10;
    }
    public double getRot(){
        return rot.getCurrentPosition()*TICKS_PER_DEG;
    }

    public void update(){
        goTo((rotTarget));
        packet.put("rotTarget", rotTarget);
//        packet.put("vel", rot.getVelocity());
//        packet.put("pos", rot.getCurrentPosition());
    }
}
