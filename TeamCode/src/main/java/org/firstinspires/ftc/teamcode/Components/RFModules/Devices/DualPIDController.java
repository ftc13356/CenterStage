package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.voltage;

import static java.lang.Double.NaN;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Components.TelescopicArm;

@Config
public class DualPIDController {
    public static double x1 = 0;
    DcMotorEx ext, ext2, rot, extEnc, rotEnc;
    public static double  A_OFF = -9, MAX=31.3, MIN=0
            , ROTMAX = 158, ROTMIN = 0, TICKS_PER_IN = 0.001821464277011343*4*31/79*30/35, TICKS_PER_DEG = 380/8192.0,P=0.2,D=0.0005, rP = 0.03 , rP2 =0.04, rD2= 3.5
            , rD =0.6 , rF = 0.4, G = 0.3,rG = 0.13, rG2 = 0.24, HORIZ_LIM = 27.2
            ,TEST_LEN = 0, MAX_SPEED = 223*751.8/60, MULT = -1, MULT2=-1;
    boolean mid=true, voltScaled = false;
    double TICKS_PER_RAD = TICKS_PER_DEG*PI/180;
    double targetExt, targetRot, middle, middleRot, trueTargExt, trueTargRot, lastPower=-0.1, curExt, curRot, vel, rotVel;
    public DualPIDController() {

        ext = op.hardwareMap.get(DcMotorEx.class, "extendMotor");
        ext2 = op.hardwareMap.get(DcMotorEx.class, "extendMotor2");
        extEnc = op.hardwareMap.get(DcMotorEx.class, "motorRightFront");
        rot = op.hardwareMap.get(DcMotorEx.class, "rotateMotor"); 
        rotEnc = op.hardwareMap.get(DcMotorEx.class, "motorRightBack");
        if(!isTeleop) {
            extEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rotEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        rot.setDirection(DcMotorSimple.Direction.REVERSE);
        ext2.setDirection(DcMotorSimple.Direction.REVERSE);
        ext.setDirection(DcMotorSimple.Direction.REVERSE);
        mid=true;
        middle=0;
        middleRot = 0;
        curExt =0;
        curRot = 0;
        vel =0;
        rotVel = 0;
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
        extension = min(max(extension,MIN),MAX);
        rotation = min(max(rotation,ROTMIN),ROTMAX);
        targetExt = extension;
        targetRot = rotation;
        curRot = -rotEnc.getCurrentPosition();
        curExt = -extEnc.getCurrentPosition() + (.5+x1)*curRot*TICKS_PER_DEG*2786.2/360;
        if((targetExt+10)*cos(curRot*TICKS_PER_RAD)>HORIZ_LIM){
            extension = HORIZ_LIM/cos(curRot*TICKS_PER_RAD)-10;
        }
        double err = extension - curExt*TICKS_PER_IN;
        double d = extEnc.getVelocity()*TICKS_PER_IN;
        double rd = rotEnc.getVelocity()*TICKS_PER_DEG;
        vel = d;
        rotVel = rd*-1;
        if((extension < curExt * TICKS_PER_IN) || abs(rotation - curRot*TICKS_PER_DEG + rd *0.3) < 30){
            ext.setPower(MULT*(P*err+D*d+G*Math.sin(curRot*TICKS_PER_RAD)));
            ext2.setPower(MULT2*(P*err+D*d+G*Math.sin(curRot*TICKS_PER_RAD)));
        }
        double rErr = rotation - curRot*TICKS_PER_DEG;
        double r = curExt*TICKS_PER_IN/MAX;
        double gScale  = 1;

        double power = 0;
        if(curExt*TICKS_PER_IN<19 || extension > 19){
            power = 13/voltage*((rP+rP2*r)*rErr+.001*(rD+rD2*r)*rd+Math.cos(curRot*TICKS_PER_RAD+(A_OFF+6*r)*PI/180)*(rG+ rG2*r));
        }
        else{
            power = Math.cos(curRot*TICKS_PER_RAD+(A_OFF+6*r)*PI/180)*(rG+ rG2*r);
        }
//        if(signum(rd) != signum(power)){
//            gScale = 1/(1-abs(rd/MAX_SPEED/TICKS_PER_DEG));
//        }
        power*=gScale;
//        packet.put("powab4rF",power);
        if(abs(rd)<0.5 && abs(rErr)>1 && (curRot<90|| abs(rErr) > 3)  && (curRot*TICKS_PER_DEG>10||targetRot>1)){
            power+=rF*signum(rErr);
        }
        if(abs(rErr)<10&&rd==0&&targetRot==0||lastPower==0&&targetRot==0)
            power=0;
        rot.setPower(power);
        lastPower = power;
        if(power ==0 &&lastPower==0&& rd==0 && targetRot ==0 && curRot!=0) {
            rotEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        packet.put("powa",power);
        packet.put("rD", abs(rd));
        packet.put("rErr", abs(rErr));
        packet.put("rGcostheta", rG*Math.cos(curRot*TICKS_PER_RAD));
        packet.put("rF", rF*signum(rErr));
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
        goTo(extension,rotation);
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
