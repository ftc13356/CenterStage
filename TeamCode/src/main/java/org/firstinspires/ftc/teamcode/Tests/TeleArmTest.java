package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.DualPIDController;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
@Autonomous

@Config
public class TeleArmTest extends LinearOpMode {
    public static double TARGET_ROT = 0, TARGET_EXT = 0, MID = 0, MID_ROT = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this,false);
        DualPIDController teleArm = new DualPIDController();
        waitForStart();
        while(!isStopRequested()&&opModeIsActive()){
            teleArm.goTo(TARGET_EXT,TARGET_ROT);
            packet.put("tar_rot", TARGET_ROT);
            packet.put("tar_ext", TARGET_EXT);
            packet.put("cur_rot", teleArm.getRot());
            packet.put("cur-ext", teleArm.getExt());
            robot.update();
        }
        stop();

    }
}
