package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

public class IDRobot extends BasicRobot{
    Claw claw;
    CVMaster cv;
    Flip flip;
    Follower follower;
    TelescopicArm arm;
    Twist twist;
    public IDRobot(LinearOpMode opMode, boolean p_isTeleop) {
        super(opMode, p_isTeleop);
        claw = new Claw();
        cv = new CVMaster();
        flip = new Flip();
        follower = new Follower(op.hardwareMap);
        arm = new TelescopicArm();
        twist = new Twist();
    }
}
