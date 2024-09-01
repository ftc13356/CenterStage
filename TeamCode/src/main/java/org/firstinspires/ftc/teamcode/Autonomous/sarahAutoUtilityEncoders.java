package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class sarahAutoUtilityEncoders{
    public DcMotor backLeft;
    public DcMotor backRight;
    private HardwareMap hardwareMap;
    public int ticks = 200; //too lazy to get the real ones rn
    //initialized backRight and backLeft as encoder thingise??? code doesn't actualyl do anything with it tho

    public sarahAutoUtilityEncoders(HardwareMap hwMap) {
        hardwareMap = hwMap;
        backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight = hardwareMap.get(DcMotor.class, "motorRightBack");
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveForward(double distance){
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setPower(1,1);
    }

    public void turn(double degrees){
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setPower(.5,.5);
        setPower(0,0);
    }

    public void setPower(double powerLeft, double powerRight){
        backLeft.setPower(powerLeft);
        backRight.setPower(powerRight);
    }
}