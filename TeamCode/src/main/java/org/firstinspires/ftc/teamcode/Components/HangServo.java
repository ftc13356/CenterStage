package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.hardware.CRServoImplEx;


public class HangServo {
    CRServoImplEx lHang, rHang;
    public HangServo(){
        lHang = op.hardwareMap.get(CRServoImplEx.class, "leftHangServo");
        rHang = op.hardwareMap.get(CRServoImplEx.class, "rightHangServo");
    }
    public void setPower(double power){
        lHang.setPower(power);
        rHang.setPower(-power);
    }
}
