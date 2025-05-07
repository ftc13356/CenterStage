package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHBUCKET_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.LOWBUCKET_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.voltage;

import static java.lang.Double.NaN;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

import android.animation.RectEvaluator;

import com.acmerobotics.dashboard.config.Config;
import com.google.gson.typeadapters.RuntimeTypeAdapterFactory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Config
public class DualPIDController {
    public static double x1 = 0;
    DcMotorEx ext, ext2, rot, extENC, rotEnc;
    Encoder extEnc;
//    public static double  A_OFF = -15, MAX=31.5, MIN=0
//            , ROTMAX = 162, ROTMIN = 0, TICKS_PER_IN = 0.001821464277011343*4*31/79*30/35, TICKS_PER_DEG = 380/8192.0,P=0.2,D=0.02, rP = 0.025 , rP2 =0.03, rD2= 2
//            , rD = .25 , rF = 0.4, G = 0.3,rG = 0.2, rG2 = 0.35, HORIZ_LIM = 27.2
//            ,TEST_LEN = 0, MAX_SPEED = 223*751.8/60, MULT = -1, MULT2=-1;

    public static double  A_OFF = -8, MAX=29, MIN=0
            , ROTMAX = 170, ROTMIN = 0, TICKS_PER_IN = 6.501950585175553e-4, TICKS_PER_DEG = 380/8192.0,P=0.15, S = 0.15, S2 = 0.07 ,D=0.005, rP = 0.009, rP2 =0.01, rD2= .3
            , rD = .2 , rF = 0.4, G = 0.2,rG = 0.15, rG2 = 0.32, HORIZ_LIM = 28.2
            ,TEST_LEN = 0, MAX_SPEED = 223*751.8/60, MULT = -1, MULT2=-1, SPECIPOWER = -0.05, rFH = 0.05, rF0 = 0.8, rG0= .1;
    boolean mid=true, voltScaled = false;
    double TICKS_PER_RAD = TICKS_PER_DEG*PI/180, lastManualTime = -100;
    double targetExt, targetRot, middle, middleRot, trueTargExt, trueTargRot, lastPower=-0.1, curExt, curRot, vel, rotVel;
    public DualPIDController() {
        ext = op.hardwareMap.get(DcMotorEx.class, "extendMotor");
        ext2 = op.hardwareMap.get(DcMotorEx.class, "extendMotor2");
        extENC = op.hardwareMap.get(DcMotorEx.class, "motorRightFront");
        rot = op.hardwareMap.get(DcMotorEx.class, "rotateMotor"); 
        rotEnc = op.hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        extEnc = new Encoder(extENC);
        if(!isTeleop) {
            rotEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ext2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extENC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extENC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        lastManualTime = -100;
        rot.setDirection(REVERSE);
//        rotEnc.setDirection(REVERSE);
        ext2.setDirection(REVERSE);
        ext.setDirection(REVERSE);
        mid=true;
        middle=0;
        middleRot = 0;
        curExt =0;
        curRot = 0;
        vel =0;
        rotVel = 0;
        targetExt=0;
        targetRot=0;
//        rP = 0.012; rP2 =0.02;rD2= 2;
//        rD = 0.6; rG = 0.07;
//        rG2 = 0.6;
        if(!voltScaled) {
//            rP*= 13 / voltage;
//            rP2*= 13 / voltage;
//            rG *= 13 / voltage;
//            rG2 *=  13 / voltage;
//            rD *= 13 / voltage;
//            rD2 *=  13 / voltage;
            voltScaled = true;
        }
    }

    public void goTo(double extension, double rotation){
        if(abs(time - lastManualTime)>3) {
            extension = min(max(extension, MIN), MAX);
            rotation = min(max(rotation, ROTMIN), ROTMAX);
            targetExt = extension;
            targetRot = rotation;
            curRot = rotEnc.getCurrentPosition();
            curExt = -extEnc.getCurrentPosition() + (x1) * curRot * TICKS_PER_DEG * 2786.2 / 360;
            if ((targetExt + 10) * cos(curRot * TICKS_PER_RAD) > HORIZ_LIM) {
                extension = HORIZ_LIM / cos(curRot * TICKS_PER_RAD) - 10;
            }
            double err = extension - curExt * TICKS_PER_IN;
            double d = extEnc.getCorrectedVelocity() * TICKS_PER_IN;
            double rd = -rotEnc.getVelocity() * TICKS_PER_DEG;
            vel = d;
            rotVel = rd * -1;
            if ((extension < curExt * TICKS_PER_IN) || abs(rotation - curRot * TICKS_PER_DEG + rd * 0.3) < 50) {
                double signum = 0;
                if(abs(targetExt-curExt*TICKS_PER_IN)>0.5 && abs(vel)<2){
                    signum = signum(targetExt-curExt*TICKS_PER_IN)*S;
                }
                else if(abs(targetExt-curExt*TICKS_PER_IN)>0.5){
                    signum = signum(targetExt-curExt*TICKS_PER_IN)*S2;
                }
                ext.setPower(MULT * (P * err + D * d + G * Math.sin(curRot * TICKS_PER_RAD) + signum));
                ext2.setPower(MULT2 * (P * err + D * d + G * Math.sin(curRot * TICKS_PER_RAD) +signum));
            }
            double rErr = rotation - curRot * TICKS_PER_DEG;
            double r = curExt * TICKS_PER_IN / MAX;
            double gScale = 1 / (1 - Math.max(-rd / 400, 0));

            double power = 0;
            if (curExt * TICKS_PER_IN < 23 || extension > 23 || true) {
                power = (13 / voltage) * (((rP + rP2 * r) * rErr + .001 * (rD + rD2 * r) * rd + Math.cos(curRot * TICKS_PER_RAD + (A_OFF - 2 * r) * PI / 180) * (rG + rG2 * r)) * gScale);
                if (targetRot != 29) {
                    power -= (13 / voltage) * Math.cos(curRot * TICKS_PER_RAD + (A_OFF - 2 * r) * PI / 180) * (rG0) * gScale;
                }
            } else {
                power = Math.cos(curRot * TICKS_PER_RAD + (A_OFF + 3 * r) * PI / 180) * (rG + rG2 * r) - .1;
            }
//        if(signum(rd) != signum(power)){
//            gScale = 1/(1-abs(rd/MAX_SPEED/TICKS_PER_DEG));
//        }
//        power*=gScale;
//        packet.put("powab4rF",power);
            if (abs(rd) < 1 && abs(rErr) > 2) {
                if (targetRot == HIGHBUCKET_PITCH_POS || targetRot == LOWBUCKET_PITCH_POS) {
                    power += rFH * signum(rErr);
                } else if (targetRot == 0) {
                    power += rF0 * signum(rErr);
                } else {
                    power += rF * signum(rErr);
                }
            }
            if (abs(rErr) < 10 && targetRot == 0 || lastPower == 0 && targetRot == 0)
                power = 0;
            if (abs(rErr) < 10 && targetRot == ROTMAX) {
                power = SPECIPOWER;
            }
            rot.setPower(power);
            lastPower = power;
            if (power == 0 && lastPower == 0 && rd == 0 && targetRot == 0 && curRot != 0 && isTeleop) {
                rotEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rotEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            packet.put("powa", power);
            packet.put("rD", abs(rd));
            packet.put("rErr", abs(rErr));
            packet.put("rGcostheta", rG * Math.cos(curRot * TICKS_PER_RAD));
            packet.put("rF", rF * signum(rErr));
        }
    }
    public void goTo(double extension, double rotation, double middle){
        if(middle != this.middle || targetExt != min(max(extension,MIN),MAX) || targetRot != min(max(rotation,ROTMIN),ROTMAX))
            mid=false;
        if(!mid && abs(getExt() - middle) < 3)
            mid=true;
        if(!mid) {
            extension = middle;
            this.middle = middle;
        }
        goTo(extension,rotation);
    }
    public void goTo(double extension, double rotation, double middle, double middleRot){
        if(mid&& (middle != this.middle || middleRot != this.middleRot || targetExt != min(max(extension,MIN),MAX) || targetRot != min(max(rotation,ROTMIN),ROTMAX)))
            mid=false;
        if(!mid && abs(getExt() - middle) < 5 && abs(getRot()-middleRot)<10)
            mid=true;
        trueTargExt = extension;
        trueTargRot = rotation;
        if(!mid) {
            extension = middle;
            rotation = middleRot;
            this.middle = middle;
            this.middleRot = middleRot;
        }
        else{
            targetRot = rotation;
            targetExt = extension;
        }
        packet.put("isMid", mid);
        packet.put("middled", abs(getExt() - middle) < 5);
        packet.put("middleRotted", abs(getRot()-middleRot)<10);
        packet.put("middleRot", middleRot);
        packet.put("middleRotCurrent", getRot());


        goTo(extension,rotation);
    }
    public void setPowers(double exte, double rota){
        ext.setPower(exte);
        ext2.setPower(exte);
        rot.setPower(rota);
        lastManualTime = time;
    }
    public double getRot(){
        return curRot*TICKS_PER_DEG;
    }
    public double getExt(){
        return curExt*TICKS_PER_IN;
    }
    public double getTargetExt(){
        return targetExt;
    }
    public double getTargetRot(){
        return targetRot;
    }
    public double getTrueTargExt(){return trueTargExt;}

    public double getTrueTargRot() {
        return trueTargRot;
    }
    public void resetMid(){
        this.middle = 0;
        this.middleRot = 0;
        mid = true;
    }

    public double getMiddle(){return middle;}
    public double getMiddleRot(){return  middleRot;}
    public boolean isMid(){return mid;}

    public double getVel(){return vel;}
    public double getRotVel(){return rotVel;}
    public double getExtPosition(){
        return extEnc.getCurrentPosition();
    }

    public double getRotPosition(){
        return -rotEnc.getCurrentPosition();
    }

}
