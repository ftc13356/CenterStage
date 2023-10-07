package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.RFMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.text.DecimalFormat;
import java.util.ArrayList;

@TeleOp(name = "ButterflyTest")
public class ButterflyTest extends LinearOpMode {
    private final double [] OFFSETS = {0.005,-0.005,0.01,0.02 };
    private boolean isButtered = false;
    private ArrayList<Servo> servos;
    DecimalFormat df = new DecimalFormat("0.00");


    public void runOpMode(){
        double lastSwitchTime = 0;
        BasicRobot robot = new BasicRobot(this, true);
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        logger2.log(RFLogger.Severity.INFO, "Program Name: " + RFLoggerTest.class.getSimpleName());
        logger2.log(RFLogger.Severity.CONFIG, "Mec drive init");
        servos = new ArrayList<>();
        servos.add(hardwareMap.servo.get("servoLeftFront"));
        servos.add(hardwareMap.servo.get("servoLeftBack"));
        servos.add(hardwareMap.servo.get("servoRightFront"));
        servos.add(hardwareMap.servo.get("servoRightBack"));
        logger2.log(RFLogger.Severity.CONFIG, "Servos hardware mapped");
        toggleServos();
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            if(!isButtered){
                drive.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
                logger2.log(RFLogger.Severity.INFO, "non-butter gamepad stick values: left_y: " + df.format(gamepad1.left_stick_y) + " | left_x: " + df.format(gamepad1.left_stick_x) + " | right_x: " + df.format(gamepad1.right_stick_x));
            }
            else{
                double y = gamepad1.left_stick_y;
                double a = -gamepad1.right_stick_x;
                telemetry.addData("rightStckX", a);
                double[] powers = {y+a,y+a,y-a,y-a};
                drive.setMotorPowers(powers[0],powers[1],powers[2],powers[3]);
                logger2.log(RFLogger.Severity.INFO, "Motor Powers butter: LF|RF|LB|RB = " + df.format(powers[0]) + " | " + df.format(powers[1])  + " | " + df.format(powers[2])  + " | " + df.format(powers[3]));
            }
            if(time>lastSwitchTime+1.0 && gamepad1.a){
                isButtered=!isButtered;
                toggleServos();
                lastSwitchTime=time;
            }
            robot.update();
        }
        stop();
    }
    public void toggleServos(){
        if(isButtered){
            for(int i=0;i<4;i++){
                double BUTTERED_POSITION = 0.6;
                servos.get(i).setPosition(BUTTERED_POSITION +OFFSETS[i]);
                logger2.log(RFLogger.Severity.CONFIG, "Butter servos down at pos: " + BUTTERED_POSITION);
            }
        }
        else{
            for(int i=0;i<4;i++){
                double INIT_POSITION = 1.0;
                servos.get(i).setPosition(INIT_POSITION);
                logger2.log(RFLogger.Severity.CONFIG, "Butter servos up at pos: " + INIT_POSITION);
            }
        }
    }
}
