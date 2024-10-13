package org.firstinspires.ftc.teamcode.Components.ExtendoArm;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import static java.lang.Double.NaN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
public class DualPIDController {
    DcMotorEx ext, rot;
    double MAX=2000, MIN=0, TICKS_PER_IN = 1.0/50, TICKS_PER_DEG = 1.0/50,P=0.05,I = 0,D=0.02, rP = 0.05, rD = 0.02,G = 0.5,rG = 0.1;
    double extErr = 0, degErr = 0;

    boolean mid = false;
    public DualPIDController() {
        ext = (DcMotorEx) op.hardwareMap.dcMotor.get("extendMotor");
        rot = (DcMotorEx) op.hardwareMap.dcMotor.get("rotateMotor");
        ext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void goTo(double extension, double rotation){
        double err = extension - ext.getCurrentPosition()*TICKS_PER_IN;
        double d = 0;
        if(!Double.isNaN(extErr))
            d = err-extErr;
        ext.setPower(P*err+D*d+G*Math.sin(rot.getCurrentPosition()*TICKS_PER_DEG));
        double rErr = rotation - rot.getCurrentPosition()*TICKS_PER_DEG;
        double rd = 0;
        if(!Double.isNaN(degErr))
            rd = rErr-degErr;
        rot.setPower(rP*ext.getCurrentPosition()*TICKS_PER_IN*rErr+rD*ext.getCurrentPosition()*TICKS_PER_IN*rd+rG*Math.sin(rot.getCurrentPosition()*TICKS_PER_DEG)*ext.getCurrentPosition()*TICKS_PER_IN);
    }
    public void goTo(double extension, double rotation, double middle){
        if(!mid){
            extension = middle;
        }
        double err = extension - ext.getCurrentPosition()*TICKS_PER_IN;
        double d = 0;
        if(!Double.isNaN(extErr))
            d = err-extErr;
        ext.setPower(P*err+D*d+G*Math.sin(rot.getCurrentPosition()*TICKS_PER_DEG));
        double rErr = rotation - rot.getCurrentPosition()*TICKS_PER_DEG;
        double rd = 0;
        if(!Double.isNaN(degErr))
            rd = rErr-degErr;
        rot.setPower(rP*ext.getCurrentPosition()*TICKS_PER_IN*rErr+rD*ext.getCurrentPosition()*TICKS_PER_IN*rd+rG*Math.sin(rot.getCurrentPosition()*TICKS_PER_DEG)*ext.getCurrentPosition()*TICKS_PER_IN);
        if(!mid && ext.getCurrentPosition()*TICKS_PER_IN - middle < 1){
            mid=true;
        }
    }
}
