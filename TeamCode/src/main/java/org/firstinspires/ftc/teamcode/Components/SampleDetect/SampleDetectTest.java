package org.firstinspires.ftc.teamcode.Components.SampleDetect;

import static org.firstinspires.ftc.teamcode.Components.CVMaster.isHsvTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.IDRobot;

@Autonomous
public class SampleDetectTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IDRobot robot = new IDRobot(this, false);
        isHsvTest = false;
        CVMaster cam = new CVMaster();
        cam.startStreamin();
        waitForStart();
        cam.swapYellow();
        while(opModeIsActive()&&!isStopRequested()){
            robot.arm.goTo(TelescopicArm.ArmStates.AUTO_GRAB);
            robot.update();
        }
        isHsvTest = false;
        stop();
    }
}
