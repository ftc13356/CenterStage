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
@TeleOp(name = "berrysimple")
public class verySimple extends LinearOpMode {
    private int test = 0;
    private boolean pressed = false;
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        while (!isStopRequested() && getRuntime() < 90) {
            if(gamepad1.right_bumper && pressed == false){
                test+=1000;
                pressed = true;
            }
            if(!gamepad1.right_bumper && pressed == true){
                pressed = false;
            }
            telemetry.addData("test",test);
            telemetry.update();
        }
        idle();
    }
}