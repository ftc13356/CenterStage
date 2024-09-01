package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class sarahAutoUtility {
    public DcMotor backLeft;
    public DcMotor backRight;
    private HardwareMap hardwareMap;

    public sarahAutoUtility(HardwareMap hwMap) {
        hardwareMap = hwMap;
        backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
        backRight = hardwareMap.get(DcMotor.class, "motorRightBack");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void moveForward(double distance){
        ElapsedTime elapsedTime = new ElapsedTime();
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setPower(1,1);
        elapsedTime.reset();
        while(elapsedTime.time() < 2.5){
        }
        setPower(0,0);
    }

    public void turn(double degrees){
        ElapsedTime elapsedTime = new ElapsedTime();
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setPower(.5,.5);
        elapsedTime.reset();
        while(elapsedTime.time() < 3){
        }
        setPower(0,0);
    }

    public void setPower(double powerLeft, double powerRight){
        backLeft.setPower(powerLeft);
        backRight.setPower(powerRight);
    }
}
