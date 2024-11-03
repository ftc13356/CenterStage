package org.firstinspires.ftc.teamcode.TeleOp;

import android.app.appsearch.PackageIdentifier;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class TestArmTeleOP extends LinearOpMode {
    public static double G = 0.3, G2 = 0.2, INIT_LEN = 0, INIT_POS = 0.85, GRAB_POS = 1, IN_GRAB_POS = 0.9, OPEN_POS = .85;
    public static boolean useManual = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        Servo extenServo = hardwareMap.get(Servo.class, "hawkServo");
        Servo clawServo = hardwareMap.get(Servo.class, "tuahServo");
        double angle = 0;
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        double len = INIT_LEN, lastTime = 0, pos = INIT_POS, lastTime2 = 0;
        extenServo.setPosition(INIT_LEN);
        clawServo.setPosition(INIT_POS);
        waitForStart();
        while (!isStopRequested()&&opModeIsActive()){
            angle = armMotor.getCurrentPosition()/751.8 * 2 * Math.PI;
            double powa = 0.3*(gamepad1.right_trigger - gamepad1.left_trigger);
            if(gamepad1.a && getRuntime()-lastTime2>0.2){
                lastTime2 = getRuntime();
                if(len==0){
                    len = 0.5;
                    extenServo.setPosition(len);
                }
                else if(len == 0.5){
                    len = 1;
                    extenServo.setPosition(len);
                }
                else{
                    len=0;
                    extenServo.setPosition(len);
                }
            }
            if(gamepad1.b && getRuntime()-lastTime2>0.2){
                lastTime2 = getRuntime();
                if(pos!=GRAB_POS){
                    pos = GRAB_POS;
                    clawServo.setPosition(pos);
                }
                else{
                    pos=OPEN_POS;
                    clawServo.setPosition(OPEN_POS);
                }
            }
            if(gamepad1.x && getRuntime()-lastTime2>0.2){
                lastTime2 = getRuntime();
                if(pos!=IN_GRAB_POS){
                    pos = IN_GRAB_POS;
                    clawServo.setPosition(pos);
                }
                else{
                    pos=OPEN_POS;
                    clawServo.setPosition(OPEN_POS);
                }
            }
            if(!useManual){
                powa=0;
            }
            telemetry.addData("power", powa);
            telemetry.addData("ticks", armMotor.getCurrentPosition());
            telemetry.addData("angle", angle);
            telemetry.addData("len", len);
            telemetry.update();
            armMotor.setPower(Math.cos(angle)*(G+len*G2) + powa);

        }
    }
}
