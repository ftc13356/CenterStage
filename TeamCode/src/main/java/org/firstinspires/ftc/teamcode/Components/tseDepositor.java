package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class tseDepositor {
    // Define class members
    CRServo tseCrServo;
    LinearOpMode op;
    ElapsedTime et;
    long initialTime;
    long retractTime;
    double reversePower;
    static final long FORWARD_ROTATION_PER_INCH = 147;
    static final long REVERSE_ROTATION_PER_INCH = 114;

    public tseDepositor(LinearOpMode opMode) {
        op = opMode;
        tseCrServo = opMode.hardwareMap.get(CRServo.class, "crtsedepositer");
        et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        initialTime = retractTime = 0;
        reversePower = 0.0;

    }
    public void moveTseDepositerTape(String name, int inch,  int forward) {
        String caption;

        tseCrServo.setPower(0);
        caption = "Servo " + name + " " + forward + " " +inch ;
        op.telemetry.addData(caption, " Moving ");
        op.telemetry.update();


        if (forward != 0) {
            tseCrServo.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            tseCrServo.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        tseCrServo.setPower(1.0);
        if (forward == 1){
            op.sleep(FORWARD_ROTATION_PER_INCH * inch);
        } else {
            op.sleep(REVERSE_ROTATION_PER_INCH * inch);
        }
        tseCrServo.setPower(0);
    }
    public void setTseCrServoPower(double power) {
        if (power == 0.0) {
            retractTime = et.now(TimeUnit.MILLISECONDS) - initialTime;
            op.telemetry.addData("tse zeropower", " retacttime "+retractTime);
            op.telemetry.update();
        } else { //power non zero means servo is moving
            if (initialTime == 0||initialTime<retractTime) {
                initialTime = et.now(TimeUnit.MILLISECONDS);
            }
            reversePower = power;

        }
        tseCrServo.setPower(-power);
    }

    public void retract() {
        long  retractTimeActual = (retractTime);
        tseCrServo.setDirection(DcMotorSimple.Direction.FORWARD);
        tseCrServo.setPower(reversePower*0.683);//12.37 and 13.38 volltage it is working
        op.telemetry.addData("tse retract", " retacttime "+ retractTimeActual + reversePower);
        op.telemetry.update();
        op.sleep(retractTimeActual);
        setTseCrServoPower(0);
    }

}