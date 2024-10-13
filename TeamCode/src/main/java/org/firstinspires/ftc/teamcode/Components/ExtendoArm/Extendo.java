package org.firstinspires.ftc.teamcode.Components.ExtendoArm;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Extendo {
    DcMotorEx motor;
    double MAX, MIN;
    public Extendo(String p_motorName, DcMotorSimple.Direction p_motorDirection, boolean p_resetPos, double p_maxtick,
                   double p_mintick) {
        motor = (DcMotorEx) op.hardwareMap.dcMotor.get(p_motorName);
        motor.setDirection(p_motorDirection);
        if (p_resetPos) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        MAX = p_maxtick;
        MIN = p_mintick;

        logger.createFile("/MotorLogs/RFMotor" + p_motorName, "Runtime    Component               " +
                "Function               Action");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void goTo(double tickPosition){
        //check preexisting target
        //apply phase power
    }
}
