package org.firstinspires.ftc.teamcode.TeleOp.Comp;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@TeleOp
public class ManualSlideReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, true);
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "extendMotor");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "extendMotor2");
        DcMotorEx motor3 = hardwareMap.get(DcMotorEx.class, "rotateMotor");
        DcMotorEx extEnc = hardwareMap.get(DcMotorEx.class, "motorRightFront");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            double multiply = 0.3;
            if(gamepad1.a){
                multiply = 1;
            } else{
                multiply = 0.3;
            }
            motor.setPower(multiply*(-gamepad1.right_trigger+gamepad1.left_trigger));
            motor2.setPower(multiply*(-gamepad1.right_trigger+gamepad1.left_trigger));
            motor3.setPower((gamepad1.right_stick_y));
            packet.put("ext", extEnc.getCurrentPosition());
            robot.update();
        }
    }
}
