package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class sarahAutoUtility {
    public DcMotor backRight, backLeft;

    public void moveForward(double distance){
        ElapsedTime elapsedTime = new ElapsedTime();
        backRight.setPower(distance);
        backLeft.setPower(distance);
        if(elapsedTime.time() > 2.5){
            backRight.setPower(0);
            backLeft.setPower(0);
        }
    }

    public void turn(double degrees){

    }
}