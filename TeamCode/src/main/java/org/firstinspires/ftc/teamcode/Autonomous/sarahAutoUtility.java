package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap; //idk what this is

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class sarahAutoUtility {
    public DcMotor backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
    public DcMotor backRight = hardwareMap.get(DcMotor.class, "motorRightBack");

    public void moveForward(double distance){ //timer
        ElapsedTime elapsedTime = new ElapsedTime();
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setPower(1,1);
        while(elapsedTime.time() < 2.5){ //timer
            setPower(0,0);
        }
    }

    public void turn(double degrees){
        ElapsedTime elapsedTime = new ElapsedTime();
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD); //leftmotor rotates forward and rightmotor rotates backward
        setPower(.5,.5);
        while(elapsedTime.time() < 3){ //timer
            setPower(0,0);
        }
    }

    public void setPower(double powerLeft, double powerRight){
        backLeft.setPower(powerLeft);
        backRight.setPower(powerRight);
    }
}