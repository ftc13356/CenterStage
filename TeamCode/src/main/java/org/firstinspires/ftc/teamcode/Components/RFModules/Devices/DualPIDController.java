package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

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
public class DualPIDController {
    DcMotorEx ext, rot;
    public static double  MAX=20, MIN=0, ROTMAX = 100, ROTMIN = -10, TICKS_PER_IN = 20, TICKS_PER_DEG = 360/537.7,P=0,D=0, rP = 0.016, rP2 =0.016,rD2= 1.2, rD = 0.7,G = 0,rG = 0.1, rG2 = 0.1,middle,TEST_LEN = 10;
    boolean mid;
    double TICKS_PER_RAD = TICKS_PER_DEG*PI/180;
    public DualPIDController() {
        ext = (DcMotorEx) op.hardwareMap.dcMotor.get("extendMotor");
        rot = (DcMotorEx) op.hardwareMap.dcMotor.get("rotateMotor");
        ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rot.setDirection(DcMotorSimple.Direction.REVERSE);
        mid=true;
        middle=-100;
    }

    public void goTo(double extension, double rotation){
        extension = min(max(extension,MIN),MAX);
        rotation = min(max(rotation,ROTMIN),ROTMAX);
        double err = extension - ext.getCurrentPosition()*TICKS_PER_IN;
        double d = ext.getVelocity();
        ext.setPower(P*err+D*d+G*Math.sin(rot.getCurrentPosition()*TICKS_PER_RAD));
        double rErr = rotation - rot.getCurrentPosition()*TICKS_PER_DEG;
        double rd = -rot.getVelocity()*TICKS_PER_DEG;
        double r = TEST_LEN/MAX;
        double power = (rP+rP2*r*r)*rErr+.001*(rD+rD2*r*r)*rd+Math.cos(rot.getCurrentPosition()*TICKS_PER_RAD)*(rG+ rG2*r);
        rot.setPower(power);
        packet.put("powa",power);
        packet.put("rG", rG);
        packet.put("rGcostheta", rG*Math.cos(rot.getCurrentPosition()*TICKS_PER_RAD));
    }
    public void goTo(double extension, double rotation, double middle){
        if(middle != this.middle)
            mid=false;
        if(!mid && ext.getCurrentPosition()*TICKS_PER_IN - middle < 0)
            mid=true;
        if(!mid)
            extension = middle;
        goTo(extension,rotation);
    }
    public double getRot(){
        return rot.getCurrentPosition()*TICKS_PER_DEG;
    }
    public double getExt(){
        return ext.getCurrentPosition()*TICKS_PER_IN;
    }
    public double getExtPosition(){
        return ext.getCurrentPosition();
    }

    public double getRotPosition(){
        return rot.getCurrentPosition();
    }
}
