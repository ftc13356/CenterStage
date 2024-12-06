package org.firstinspires.ftc.teamcode.SampleDetect;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CapCam;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.BradBot;

import java.util.Arrays;
import java.util.Objects;
@Config
@Autonomous
public class SamplePnpTest extends LinearOpMode {
    public static double downAng = -18;

    @Override
    public void runOpMode() throws InterruptedException {
//        BradBot robot = new BradBot(this, false, false);
//        robot.roadrun.setPoseEstimate(new Pose2d(0,0,0));
        BasicRobot robot = new BasicRobot(this,false);
        currentPose = new Pose2d(0,0,0);
        CapCam cam = new CapCam();
        cam.startStreamin();
        double lastTime = 0;
        int buhs = 0;
        double bps = 0;

        waitForStart();
        resetRuntime();
        while(opModeIsActive()&&!isStopRequested()){
//            if(getRuntime()>lastTime+5 && !Arrays.equals(cam.getCenter(), new double[]{0, 0, 0})) {
//                if(new Vector2d(cam.getCenter()[0], cam.getCenter()[1]).norm()>1) {
//                    robot.roadrun.followTrajectoryAsync(robot.roadrun.trajectoryBuilder(currentPose)
//                            .lineTo(currentPose.vec().plus(new Vector2d(cam.getCenter()[0], -cam.getCenter()[1]))).build());
//                                    packet.put("TARGET", currentPose.vec().plus(new Vector2d(cam.getCenter()[0], -cam.getCenter()[1])));
//                    cam.pnp.resetCenter();
//                    lastTime = getRuntime();
//                }
//            }


//            cam.swapBlue();
            if(!Arrays.equals(cam.getCenter(), new double[]{0, 0, 0})) {
                BasicRobot.time = getRuntime();
                buhs++;
                double[] relCent = cam.getCenter().clone();
                if (!Arrays.equals(relCent, new double[]{0, 0, 0})) {
                    while(relCent[0]==0 || relCent[1]==0 || relCent[2]<20){
                        relCent = cam.getCenter().clone();
                    }
                    cam.resetCenter();
                    double rel = relCent[0];
                    relCent[0] = relCent[2] * Math.sin(downAng * PI / 180) + relCent[0] * Math.cos(downAng * PI / 180);
                    relCent[2] = sqrt(relCent[2] * relCent[2] + rel * rel - relCent[0] * relCent[0]);
                    double angle = Math.atan2(relCent[1], relCent[2])*180/PI;

                    packet.put("relCent0", relCent[0]);
                    packet.put("relCent1", relCent[1]);
                    packet.put("relCent2", relCent[2]);
                    packet.put("TARGANGDel", angle);
                }
                packet.put("BUHPERSEC(BPS)", buhs/BasicRobot.time);
                packet.put("color", cam.getCurrent());
                cam.resetCenter();
                dashboard.sendTelemetryPacket(packet);
                packet.clearLines();
            }
            packet.put("color", cam.getCurrent());
            dashboard.sendTelemetryPacket(packet);
        }

        stop();
    }
}
