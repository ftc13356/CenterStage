package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.Line;

public class sarahAutoUtilityTimer {
    public DcMotor backLeft, backRight;
    private HardwareMap hardwareMap;
    public LinearOpMode op;

    public sarahAutoUtilityTimer(LinearOpMode opMode) {
        op = opMode;
        hardwareMap = op.hardwareMap;
        backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
        backRight = hardwareMap.get(DcMotor.class, "motorRightBack");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void moveForward(double distance){
        backLeft.setPower(.4);
        backRight.setPower(.4);
        op.sleep(700);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turn(double degrees){
        backLeft.setPower(.4);
        op.sleep(1500);
        backLeft.setPower(0);
    }
}
