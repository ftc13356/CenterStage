package org.firstinspires.ftc.teamcode.TeleOp.Comp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ManualSlideReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "extendMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            double multiply = 0.3;
            if(gamepad1.a){
                multiply = 1;
            } else{
                multiply = 0.3;
            }
            motor.setPower(multiply*(-gamepad1.right_trigger+gamepad1.left_trigger));
        }
    }
}
