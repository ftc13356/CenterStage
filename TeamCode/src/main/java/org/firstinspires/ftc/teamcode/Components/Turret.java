package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Double.NaN;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class Turret {
    DcMotorEx rot;
    public static double  ROTMAX = 100, ROTMIN = -100, TICKS_PER_DEG = 360/537.7, rP = 0, rD = 0;
    boolean mid;
    double TICKS_PER_RAD = TICKS_PER_DEG*PI/180;
    public Turret() {
        rot = (DcMotorEx) op.hardwareMap.dcMotor.get("turt");
        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rot.setDirection(DcMotorSimple.Direction.REVERSE);
        mid=true;
    }

    public void goTo(double rotation){
        rotation = min(max(rotation,ROTMIN),ROTMAX);
        double rErr = rotation - rot.getCurrentPosition()*TICKS_PER_DEG;
        double rd = -rot.getVelocity()*TICKS_PER_DEG;
        double power = (rP)*rErr+.001*(rD)*rd+Math.cos(rot.getCurrentPosition()*TICKS_PER_RAD);
        rot.setPower(power);
        packet.put("powa",power);
    }
    public double getRot(){
        return rot.getCurrentPosition()*TICKS_PER_DEG;
    }
}
