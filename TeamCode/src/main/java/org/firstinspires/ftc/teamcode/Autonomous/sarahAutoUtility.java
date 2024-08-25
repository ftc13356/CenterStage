package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class sarahAutoUtility {
    public DcMotor backLeft, backRight;

    public void moveForward(double distance){
        //currently doesn't do anything with distance but will do something with it with encoders later
        ElapsedTime elapsedTime = new ElapsedTime(); //timer
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setPower(1,1);
        if(elapsedTime.time() > 2.5){
            setPower(0,0);
        }
    }

    public void turn(double degrees){
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD); //leftmotor rotates forward and rightmotor rotates backward
        setPower(.5,.5);
    }

    public void setPower(double powerLeft, double powerRight){
        backLeft.setPower(powerLeft);
        backRight.setPower(powerRight);
    }
}