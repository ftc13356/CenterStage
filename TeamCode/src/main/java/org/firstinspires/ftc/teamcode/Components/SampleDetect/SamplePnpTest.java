package org.firstinspires.ftc.teamcode.Components.SampleDetect;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.Arrays;

@Autonomous
public class SamplePnpTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        BradBot robot = new BradBot(this, false, false);
//        robot.roadrun.setPoseEstimate(new Pose2d(0,0,0));
        BasicRobot robot = new BasicRobot(this,false);
        currentPose = new Pose2d(0,0,0);
        CVMaster cam = new CVMaster();
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

            if(gampad.readGamepad(op.gamepad1.a,"gamepad1_a", "swapCam"))
                cam.swapNext();

            if(!Arrays.equals(cam.getCenter(), new double[]{0, 0, 0})) {
                BasicRobot.time = getRuntime();
                packet.put("TARGET", currentPose.vec().plus(new Vector2d(cam.getCenter()[0], -cam.getCenter()[1])));
                buhs++;
                packet.put("BUHPERSEC(BPS)", buhs/BasicRobot.time);
                cam.resetCenter();
                dashboard.sendTelemetryPacket(packet);
                packet.clearLines();
            }
        }

        stop();
    }
}
