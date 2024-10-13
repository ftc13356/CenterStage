package org.firstinspires.ftc.teamcode.SampleDetect;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.BradBot;

@Autonomous
public class SamplePnpTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false, false);
        CameraInit cam = new CameraInit(true, this);
        cam.startStreamin();
        double lastTime = 0;
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
//            packet.put("center", cam.getCenter());
//            if(BasicRobot.time>lastTime+5) {
//                robot.roadrun.followTrajectory(robot.roadrun.trajectoryBuilder(currentPose)
//                        .lineTo(currentPose.vec().plus(new Vector2d(cam.getCenter()[0], cam.getCenter()[1]))).build());
//            }
//            robot.update();
//            robot.queuer.setFirstLoop(false);
        }

        stop();
    }
}
