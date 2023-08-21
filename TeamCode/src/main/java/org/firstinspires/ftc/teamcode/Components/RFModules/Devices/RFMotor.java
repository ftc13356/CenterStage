package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.text.DecimalFormat;
import java.util.ArrayList;

@Config
public class  RFMotor extends Motor {
    private DcMotorEx rfMotor = null;
    private VoltageSensor voltageSensor = null;
    private ArrayList<Double> coefs = null;
    private ArrayList<Double> coefs2 = null;
    private ArrayList<String> inputlogs = new ArrayList<>();
    public static double kP = 0, kD = 0, kV = 0.0006, kA = 0.000055, kG = 0.32, kS = 0.1;
    public double appliedkS = 0, kSneg = -0.1, kSpos = 0.1;
    private double maxtickcount = 0;
    private double mintickcount = 0;
    private double DEFAULTCOEF1 = 0.0001, DEFAULTCOEF2 = 0.01;
    private double lastError = 0, lastTime = 0;
    private double additionalTicks = 0;
    private double TICK_BOUNDARY_PADDING = 10, TICK_STOP_PADDING = 20;
    private double TICK_STATIC_PADDING = 20;
    private String rfMotorName;
    private double power = 0;
    private double[] targets = {0,0,0,0};
    private boolean isStopped = false;
    private double MAX_VEL = 1500, MAX_UP_VEL = 1500, MAX_DOWN_VEL = 1000, MAX_ACCEL = 6000, MAX_DOWN_ACCEL = 2000, MAX_UP_ACCEL = 6000;
    private double d0 = 0, v0 = 0, dTarget = -1, t0 = 0, tAccel = 0,  tDecel = 0, tTotal = 0, dTravel = 0, tStart = 0, dCurr = 0, vCurr = 0;
    private double sign = 0;
    private double phase = 0;

    private static final DecimalFormat df = new DecimalFormat("0.00");

    /*Initializes the motor
        Inputs:
        motorName: the name of the device | Ex:'motorRightFront'
        motorDirection: the direction of the motor | 0 for Reverse, 1 for Forward | Ex: 0
     */

    //for motors used for complex functions

    public RFMotor(String motorName, DcMotorSimple.Direction motorDirection, DcMotor.RunMode runMode,
                   boolean resetPos, ArrayList<Double> coefficients,
                   double maxtick, double mintick) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        rfMotor.setDirection(motorDirection);
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);
        coefs = coefficients;
        maxtickcount = maxtick;
        mintickcount = mintick;

        logger.createFile("/MotorLogs/RFMotor" + motorName, "Runtime    Component               " +
                "Function               Action");
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        additionalTicks = 0;
        voltageSensor = op.hardwareMap.voltageSensor.iterator().next();

        MAX_UP_VEL*= voltageSensor.getVoltage() / 13;
        MAX_UP_ACCEL*= voltageSensor.getVoltage() / 13;
        MAX_DOWN_VEL= MAX_UP_VEL + 1000;
        MAX_DOWN_ACCEL = MAX_UP_ACCEL + 2000;
    }

    //same as above but assuming motor direction is foward
    public RFMotor(String motorName, DcMotor.RunMode runMode, boolean resetPos,
                   ArrayList<Double> coefficients, double maxtick, double mintick) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        rfMotorName = motorName;
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);
        coefs = coefficients;
        maxtickcount = maxtick;
        mintickcount = mintick;

        logger.createFile("/MotorLogs/RFMotor" + motorName, "Runtime    Component               " +
                "Function               Action");
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        additionalTicks =0;
        voltageSensor = op.hardwareMap.voltageSensor.iterator().next();


        MAX_UP_VEL*= voltageSensor.getVoltage() / 13;
        MAX_UP_ACCEL*= voltageSensor.getVoltage() / 13;
        MAX_DOWN_VEL= MAX_UP_VEL + 2000;
        MAX_DOWN_ACCEL = MAX_UP_ACCEL + 6000;

    }


    //same as above but using default coefficients
    public RFMotor(String motorName, DcMotor.RunMode runMode, boolean resetPos,
                   double maxtick, double mintick) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        rfMotorName = motorName;
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);
        coefs = new ArrayList<>();
        coefs.add(DEFAULTCOEF1);
        coefs.add(DEFAULTCOEF2);
        maxtickcount = maxtick;
        mintickcount = mintick;

        logger.createFile("/MotorLogs/RFMotor" + rfMotorName, "Runtime    Component               " +
                "Function               Action");
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        voltageSensor = op.hardwareMap.voltageSensor.iterator().next();

        MAX_UP_VEL*= voltageSensor.getVoltage() / 13;
        MAX_UP_ACCEL*= voltageSensor.getVoltage() / 13;
        MAX_DOWN_VEL= MAX_UP_VEL + 1000;
        MAX_DOWN_ACCEL = MAX_UP_ACCEL + 2000;
    }

    //for chassis wheels where you only need it to spin continuously
    public RFMotor(String motorName, DcMotor.RunMode runMode, boolean resetPos) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        rfMotorName = motorName;
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);

        logger.createFile("/MotorLogs/RFMotor" + motorName, "Runtime    Component               " +
                "Function               Action");
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        voltageSensor = op.hardwareMap.voltageSensor.iterator().next();

        MAX_UP_VEL*= voltageSensor.getVoltage() / 13;
        MAX_UP_ACCEL*= voltageSensor.getVoltage() / 13;
        MAX_DOWN_VEL= MAX_UP_VEL + 1000;
        MAX_DOWN_ACCEL = MAX_UP_ACCEL + 2000;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        rfMotor.setDirection(direction);
    }

    public void setPosition(double targetPos){
        if(dTarget != targetPos){
            appliedkS = 0;
            dTarget = targetPos;
//            if(isStopped == true){
//                v0 = 0;
//            }
//            else{
//                v0 = getVelocity();
//            }
            v0 = getVelocity();
            d0 = getCurrentPosition();
            t0 = v0/MAX_ACCEL;
            sign = dTarget-d0 + v0*t0/2;//checked
            if(sign > 0){
                sign = 1;
                MAX_VEL = MAX_UP_VEL;
                MAX_ACCEL = MAX_UP_ACCEL;
//                kS = kSneg;
            }
            else{
                sign = -1;
                MAX_VEL = MAX_DOWN_VEL;
                MAX_ACCEL = MAX_DOWN_ACCEL;
//                kS = kSpos;
            }
            dTravel = Math.abs(dTarget - d0) + v0*t0/2;
            if(dTravel - pow(MAX_VEL,2)/MAX_ACCEL > 0){
                tTotal = 2*MAX_VEL/MAX_ACCEL + (dTravel - pow(MAX_VEL,2)/MAX_ACCEL)/MAX_VEL;
            }
            else{
                tTotal = 2*sqrt(dTravel/MAX_ACCEL);
            }
            tAccel = min(tTotal/2, MAX_VEL/MAX_ACCEL);
            tDecel = tTotal - tAccel;
            tStart = time;
        }
    }

    public double getv0(){
        return v0;
    }

    public double getd0(){
        return d0;
    }

    public double getApplied(){ return appliedkS; }

    public void update(){
        dCurr = getCurrentPosition();
        vCurr = getVelocity();
        sign = dTarget-d0 + v0*t0/2;
        if(sign > 0){
            sign = 1;
        }
        else{
            sign = -1;
        }
        double[] targetMotion = getTargetMotion();
        double[] error = {dCurr - targetMotion[0], vCurr - targetMotion[1]};
        if(Math.abs(dTarget-dCurr) > 20 && Math.abs(vCurr) < 2){
            appliedkS = kS*sign;
        }
        else{
            appliedkS = 0;
        }
        power = (kP * error[0] + kD * error[1] + kV * targetMotion[1] + kA * targetMotion[2]) + kG + appliedkS;
        setRawPower(power);
    }

    public double getPower(){
        return rfMotor.getPower();
    }

    public boolean profileDone(){
        if(tTotal - (time-tStart) > 0){
            return false;
        }
        else{
            return true;
        }
    }

    public double[] getStuff(double targetPos, double p_v0, double p_d0){
        if(dTarget != targetPos){
            dTarget = targetPos;
            v0 = p_v0;
            d0 = p_d0;
            t0 = v0/MAX_ACCEL; //checked
            sign = dTarget-d0 + v0*t0*0.5;//checked
            if(sign > 0){
                sign = 1;
            }
            else{
                sign = -1;
            }
            dTravel = Math.abs(dTarget - d0) + v0*t0*0.5;//checked
            if(dTravel - pow(MAX_VEL,2)/MAX_ACCEL > 0){
                tTotal = 2*MAX_VEL/MAX_ACCEL + (dTravel - pow(MAX_VEL,2)/MAX_ACCEL)/MAX_VEL;
            }
            else{
                tTotal = 2*sqrt(dTravel/MAX_ACCEL);
            }
            tAccel = min(tTotal*0.5, MAX_VEL/MAX_ACCEL);
            tDecel = tTotal - tAccel;
            tStart = time; //checked
        }
        double[] targetMotion = getTargetMotion();
//        targetMotion[3] = tTotal;
//        targetMotion[4] = tAccel;
//        targetMotion[5] = tDecel;
//        targetMotion[6] = tStart;


//        targetMotion[3] = tTotal;
//        targetMotion[4] = t0;
//        targetMotion[5] = tAccel;
//        targetMotion[6] = tDecel;
        return targetMotion;
    }

    public double[] getTimes(){
        double[] output = {tStart, tTotal, tAccel, tDecel, t0};
        return output;
    }

    public double getTarget(){
        return dTarget;
    }

    double profileTime;
    public double[] getTargetMotion() {
        profileTime = time - tStart;
        if(profileTime < tAccel - t0){
            targets[0] = d0
                    + sign*(v0*(profileTime)
                    + (profileTime)*(v0+MAX_ACCEL*(profileTime))*0.5);
            targets[1] = sign*(v0 + MAX_ACCEL*(profileTime));
            targets[2] = sign*MAX_ACCEL;
            targets[3] = profileTime;
            phase = 1;
        }
        else if(profileTime < tDecel - t0){
            targets[0] = d0
                    + sign*(v0*(tAccel-t0)*0.5
                    + (tAccel-t0)*(v0+MAX_ACCEL*(tAccel-t0))*0.5
                    + (v0+MAX_ACCEL*(tAccel-t0))*((profileTime) - tAccel + t0));
            targets[1] = sign*(v0 + MAX_ACCEL*(tAccel-t0));
            targets[2] = 0;
            targets[3] = profileTime;
            phase = 2;
        }
        else if(profileTime < tTotal - t0){
            targets[0] = d0
                    + sign*((v0*(tAccel-t0)*0.5)
                    + (tAccel-t0)*(v0+MAX_ACCEL*(tAccel-t0))*0.5
                    + (v0+MAX_ACCEL*(tAccel-t0))*((tDecel-t0) - tAccel + t0)
                    + ((profileTime) - tDecel + t0)*(v0+MAX_ACCEL*(tAccel-t0))
                    - (MAX_ACCEL*pow((profileTime) - tDecel + t0,2))*0.5);
            targets[1] = sign*(v0 + MAX_ACCEL*(tAccel-t0) - MAX_ACCEL*(profileTime-tDecel+t0));
            targets[2] = sign*(-MAX_ACCEL);
            targets[3] = profileTime;
            phase = 3;
        }
        else{
            targets[1]=0;
            targets[2]=0;
        }
        return targets;
    }
    public double getkS(){
        return appliedkS;
    }

    public void setPower(double power) {
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        logger.log("/RobotLogs/GeneralRobot", rfMotorName + ",setPower():,Setting Power: " + power, false, false);
        rfMotor.setPower(power);
//        logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Power," + (power - kP * getResistance()), false, false);
    }

    public void setRawPower(double power) {
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        getAvgResistance();
//        logger.log("/RobotLogs/GeneralRobot", rfMotorName + ",setPower():,Setting Power: " + power, false, false);
        rfMotor.setPower(power);
//        logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Power," + (power), false, false);

    }

    public void setVelocity(double velocity) {
        if (rfMotor.getVelocity() != velocity) {
//            inputlogs.add(rfMotorName);
//            inputlogs.add("setVelocity()");
//            inputlogs.add("Setting Velocity: " + velocity);
//            logger.log("/MotorLogs/RFMotor", rfMotorName + ",setVelocity()," +
//                    "Setting Velocity: " + df.format(velocity), true, true);
//            inputlogs.clear();
//            logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Velocity," + velocity);
//            logger.log("/RobotLogs/GeneralRobot", rfMotorName + "\nsetVelocity():\nSetting Velocity:" + velocity);
        }
        rfMotor.setVelocity(velocity);
    }

    public int getCurrentPosition() {
//        inputlogs.add(rfMotorName);
//        inputlogs.add("getCurrentPosition()");
//        inputlogs.add("Getting Position: " + rfMotor.getCurrentPosition());
//        inputlogs.clear();

//        logger.log("/RobotLogs/GeneralRobot", inputlogs);
//        logger.log("/MotorLogs/RFMotor" + rfMotorName, "Current Tick Count," + rfMotor.getCurrentPosition());
        return rfMotor.getCurrentPosition()+(int)additionalTicks;
    }

    public void setMode(DcMotor.RunMode runMode) {
        rfMotor.setMode(runMode);
        if (rfMotor.getMode() != runMode) {
//            inputlogs.add(rfMotorName);
//            inputlogs.add("setMode()");
//            inputlogs.add("Setting RunMode: " + runMode);
            logger.log("/MotorLogs/RFMotor", rfMotorName + ",setMode(),Setting RunMode: " + runMode,
                    true, true);
//            inputlogs.clear();

//            logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Mode," + runMode);
//            logger.log("/RobotLogs/GeneralRobot", rfMotorName + "setMode():\nSetting Mode," + runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        rfMotor.setZeroPowerBehavior(behavior);
    }

    public double getVelocity() {
        return rfMotor.getVelocity();
    }

    public double getTICK_BOUNDARY_PADDING() {
        return TICK_BOUNDARY_PADDING;
    }

    public double getTICK_STOP_PADDING() {
        return TICK_STOP_PADDING;
    }

    public void setTICK_BOUNDARY_PADDING(double p_TICK_BOUNDARY_PADDING) {
        TICK_BOUNDARY_PADDING = p_TICK_BOUNDARY_PADDING;
    }

    public void setTICK_STOP_PADDING(double p_TICK_STOP_PADDING) {
        TICK_STOP_PADDING = p_TICK_STOP_PADDING;
    }
}