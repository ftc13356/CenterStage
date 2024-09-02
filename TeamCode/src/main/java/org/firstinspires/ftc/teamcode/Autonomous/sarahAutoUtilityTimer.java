package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.Line;

public class sarahAutoUtilityTimer {
    public DcMotor backLeft;
    public DcMotor backRight;
    private HardwareMap hardwareMap;
    public LinearOpMode op;

    public sarahAutoUtilityTimer(LinearOpMode opMode) {
        op = opMode;
        hardwareMap = op.hardwareMap;
        backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
        backRight = hardwareMap.get(DcMotor.class, "motorRightBack");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveForward(double distance){
        setPower(.4,.4);
        op.sleep(5000);
        setPower(0,0);
    }

    public void turn(double degrees){
        setPower(-.15,.15);
        op.sleep(5000);
        setPower(0,0);
    }

    public void setPower(double powerLeft, double powerRight){
        backLeft.setPower(powerLeft);
        backRight.setPower(powerRight);
    }
}
