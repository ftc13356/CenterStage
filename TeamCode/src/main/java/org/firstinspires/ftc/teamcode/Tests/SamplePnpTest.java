package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;

import static java.lang.Math.atan2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.Arrays;

@Autonomous
public class SamplePnpTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        BradBot robot = new BradBot(this, false, false);
//        robot.roadrun.setPoseEstimate(new Pose2d(0,0,0));
        BasicRobot robot = new BasicRobot(this, false);
        Follower follower = new Follower(this.hardwareMap);
        currentPose = new Pose2d(0, 0, 0);
        CVMaster cv = new CVMaster();
        cv.startStreamin();
        double lastTime = 0;
        int buhs = 0;
        double bps = 0;
        boolean isAutoGrab = false;
        boolean targeted = false;
        waitForStart();
        resetRuntime();
        while (opModeIsActive() && !isStopRequested()) {
//            if(getRuntime()>lastTime+5 && !Arrays.equals(cam.getCenter(), new double[]{0, 0, 0})) {
//                if(new Vector2d(cam.getCenter()[0], cam.getCenter()[1]).norm()>1) {
//                    robot.roadrun.followTrajectoryAsync(robot.roadrun.trajectoryBuilder(currentPose)
//                            .lineTo(currentPose.vec().plus(new Vector2d(cam.getCenter()[0], -cam.getCenter()[1]))).build());
//                                    packet.put("TARGET", currentPose.vec().plus(new Vector2d(cam.getCenter()[0], -cam.getCenter()[1])));
//                    cam.pnp.resetCenter();
//                    lastTime = getRuntime();
//                }
//            }

            if (gampad.readGamepad(op.gamepad1.a, "gamepad1_a", "swapCam"))
                cv.swapNext();
            if (gampad.readGamepad(op.gamepad1.x, "gamepad1_x", "getem")) {
                isAutoGrab = true;
            }
            if (!isAutoGrab) {
                cv.resetCenter();
                targeted = false;
            }
            if (isAutoGrab) {
                double[] relCent = cv.getCenter();
                if (!Arrays.equals(relCent, new double[]{0, 0, 0})) {
                    follower.stopTeleopDrive();
                    targeted = true;
                    if (relCent[0] * relCent[0] + relCent[1] * relCent[1] < 1) {
                        isAutoGrab = false;
                        targeted = false;
                    } else {
                        Vector2d relVect = new Vector2d(relCent[0], -relCent[1]).rotated(-follower.getPose().getHeading());
                        Pose pos = follower.getPose();
                        pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                        follower.holdPoint(new BezierPoint(new Point(pos)), pos.getHeading());

                    }
                } else if (!targeted) {
                    follower.startTeleopDrive();
                    follower.setTeleOpMovementVectors(0.2, 0, 0);
                }
            }
        }

        stop();
    }
}
