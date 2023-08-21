package org.firstinspires.ftc.teamcode.Old.PowerPlay.TeleOp;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.FWDRobot;

@Config
@TeleOp(name = "MaxTuning")
public class MaxVelTuning extends LinearOpMode {
    public RFMotor slidesMotor;
    private double highestVelocity = 0;
    public void runOpMode() {
        FWDRobot robot = new FWDRobot(this, true);
        slidesMotor = new RFMotor("liftMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (!isStopRequested() && getRuntime() < 90) {
//            if(gamepad1.right_trigger != 0){
//                slidesMotor.setPower(1);
//            }
//            if(gamepad1.left_trigger != 0){
//                slidesMotor.setPower(-1);
//            }
//            if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
//                slidesMotor.setPower(0);
//            }
//            if(slidesMotor.getVelocity() > highestVelocity){
//                highestVelocity = slidesMotor.getVelocity();
//            }
//            telemetry.addData("highestVelocity", highestVelocity);
//            telemetry.update();
//            if(gamepad1.a) {
//                slidesMotor.setPosition(1000);
//            }
            if(gamepad1.y){
                slidesMotor.setPosition(1300);
            }
            if(gamepad1.b){
                slidesMotor.setPosition(0);
            }
            if(gamepad1.x){
                slidesMotor.setPosition(100);
            }
            if(gamepad1.dpad_up){
                slidesMotor.setPosition(500);
            }
            if(gamepad1.dpad_right){
                slidesMotor.setPosition(400);
            }
            if(gamepad1.dpad_down){
                slidesMotor.setPosition(300);
            }
            if(gamepad1.dpad_left){
                slidesMotor.setPosition(200);
            }
            slidesMotor.update();
            telemetry.addData("position",slidesMotor.getTargetMotion()[0]);
            telemetry.addData("velocity",slidesMotor.getTargetMotion()[1]);
            telemetry.addData("accel",slidesMotor.getTargetMotion()[2] * 0.5);
            telemetry.addData("pos", slidesMotor.getCurrentPosition());
            telemetry.addData("vel", slidesMotor.getVelocity());
            telemetry.addData("power",slidesMotor.getPower() * 1000);
            telemetry.addData("v0", slidesMotor.getv0()*100);
            telemetry.addData("d0",slidesMotor.getd0()*100);
            telemetry.addData("appliedKS", slidesMotor.getApplied()*1000);

            telemetry.update();
            robot.updateTime();
        }
        idle();
    }
}