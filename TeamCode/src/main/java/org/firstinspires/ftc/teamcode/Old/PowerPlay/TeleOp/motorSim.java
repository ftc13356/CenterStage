package org.firstinspires.ftc.teamcode.Old.PowerPlay.TeleOp;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.FWDRobot;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Config
@TeleOp(name = "motorSim")
public class motorSim extends LinearOpMode {
    public RFMotor slidesMotor;
    private boolean hasPressed = false;
    private double velAtMax = 0;
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        double maxPos = 0;
        int test = 0;
        BasicRobot robot = new BasicRobot(this, true);
        slidesMotor = new RFMotor("liftMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (!isStopRequested() && getRuntime() < 90) {
            double[] values = {0,0,0,0};
            double[] times = slidesMotor.getTimes();
            if(gamepad1.right_bumper && hasPressed == false){
                hasPressed = true;
            }
//            if(!gamepad1.right_bumper && hasPressed == true){
//                hasPressed = false;
//            }
            if(hasPressed == true){
                values = slidesMotor.getStuff(900,0,1000);
            }
            if(values[0] > maxPos){
                maxPos = values[0];
            }
            telemetry.addData("pos",values[0]);
            telemetry.addData("vel",values[1]);
            telemetry.addData("accel",values[2]);
            telemetry.addData("time",values[3]);
            telemetry.addData("maxpos",maxPos);
            telemetry.addData("Start", times[0]);
            telemetry.addData("Total", times[1]);
            telemetry.addData("Accel", times[2]);
            telemetry.addData("Decel", times[3]);
            telemetry.addData("zero", times[4]);
            if(slidesMotor.profileDone() == true){
                test = 1;
            }
            else{
                test = 0;
            }
            telemetry.addData("time", test);
//            telemetry.addData("tTotal", values[3]);
//            telemetry.addData("tAccel", values[4]);
//            telemetry.addData("tDecel", values[5]);
//            telemetry.addData("tStart", values[6]);
//            telemetry.addData("tTotal",values[3]);
//            telemetry.addData("t0",values[4]);
//            telemetry.addData("tAccel",values[5]);
//            telemetry.addData("tDecel",values[6]);
            telemetry.update();
            robot.updateTime();
        }
        idle();
    }
}