package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_EXTEND_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_PITCH_POS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;

@Autonomous
public class SpeciArmTEst extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IDRobot robot = new IDRobot(this, false);
        waitForStart();
        robot.claw.goTo(Claw.ClawStates.CLOSED);
        robot.flip.flipTo(Flip.FlipStates.SPECIMEN);
        robot.twist.twistTo(Twist.TwistStates.PARALLEL);
        while(opModeIsActive()&&!isStopRequested()){
            robot.arm.goTo(HIGHSPECIMEN_EXTEND_POS, HIGHSPECIMEN_PITCH_POS);
            robot.update();
        }
        stop();
    }
}