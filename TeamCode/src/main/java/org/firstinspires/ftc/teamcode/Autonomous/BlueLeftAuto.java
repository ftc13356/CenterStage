package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
@Autonomous(name = "BlueLeftPreParkAuto")
public class BlueLeftAuto extends LinearOpMode {
        public static int pos = 1;

        @Override
        public void runOpMode() throws InterruptedException {
            BradBot robot = new BradBot(this, false);
            robot.roadrun.setPoseEstimate(new Pose2d(-38.5,56, Math.toRadians(90)));
            TrajectorySequence[] spikePosition = new TrajectorySequence[3];

            spikePosition[0] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, 56, Math.toRadians(90)))
                    .setReversed(true)
                    .splineTo(new Vector2d(-29.5, 43),toRadians(320))
                    .addTemporalMarker(robot::done)
                    .build();
            spikePosition[1] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, 56, Math.toRadians(90)))
                    .setReversed(true)
                    .lineToLinearHeading(new Pose2d(-33.5, 32.5, toRadians(90)))
                    .addTemporalMarker(robot::done)
                    .build();
            spikePosition[2] = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, 56, Math.toRadians(90)))
                    .setReversed(true)
                    .lineToLinearHeading(new Pose2d(-43.5, 40, toRadians(90)))
                    .addTemporalMarker(robot::done)
                    .build();


            TrajectorySequence[] throughTruss = new TrajectorySequence[3];
            throughTruss[2] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(spikePosition[0].end())
                            .setReversed(true)
                            .lineToLinearHeading(new Pose2d(-40, 60, toRadians(180)))
                            .setReversed(true)
                            .splineTo(new Vector2d(13, 58), toRadians(-4))
                            .splineTo(new Vector2d(45, 40), toRadians(0))
                            .waitSeconds(2)
                            .lineToLinearHeading(new Pose2d(53, 28, toRadians(179)))
                            .addTemporalMarker(robot::done)
                            .build();
            throughTruss[1] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(spikePosition[1].end())
                            .lineToLinearHeading(new Pose2d(-39.5, 29.5, toRadians(90)))
                            .setReversed(true)
                            .lineToLinearHeading(new Pose2d(-40, 59, toRadians(180)))
                            .splineTo(new Vector2d(13, 58.5), toRadians(-4))
                            .splineTo(new Vector2d(40, 40), toRadians(0))
                            .waitSeconds(2)
                            .lineToLinearHeading(new Pose2d(53, 35, toRadians(180)))
                            .addTemporalMarker(robot::done)
                            .build();
            throughTruss[0] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(spikePosition[2].end())
                            .setReversed(true)
                            .lineToLinearHeading(new Pose2d(-40, 60, toRadians(180)))
                            .setReversed(true)
                            .splineTo(new Vector2d(14, 59), toRadians(-5))
                            .splineTo(new Vector2d(45, 44), toRadians(0))
                            .waitSeconds(2)
                            .lineToLinearHeading(new Pose2d(53.5, 43, toRadians(179)))
                            .addTemporalMarker(robot::done)
                            .build();
            TrajectorySequence[] dropAndPark = new TrajectorySequence[3];
            dropAndPark[0] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(throughTruss[0].end())
                            .lineToLinearHeading(new Pose2d(49, 58, toRadians(180)))
                            .addTemporalMarker(robot::done)
                            .build();
            dropAndPark[1] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(throughTruss[1].end())
                            .lineToLinearHeading(new Pose2d(49.5, 58, toRadians(180)))
                            .addTemporalMarker(robot::done)
                            .build();
            dropAndPark[2] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(throughTruss[2].end())
                            .lineToLinearHeading(new Pose2d(49, 58, toRadians(180)))
                            .addTemporalMarker(robot::done)
                            .build();
            while(!isStarted()){
                pos = robot.getBlueSpikePos();
                op.telemetry.addData("spike pos", pos);
                op.telemetry.update();
                robot.update();
            }
            while(!isStopRequested()&&opModeIsActive()&&!robot.queuer.isFullfilled()){
                robot.queuer.addDelay(4);
                robot.followTrajSeq(spikePosition[pos]);
                robot.queuer.addDelay(4.5);
                robot.preloadAuto();
                robot.queuer.addDelay(1.5);
                robot.followTrajSeq(throughTruss[pos]);
                robot.queuer.addDelay(6.5);
                robot.flipAuto();
                robot.loadAuto();
                robot.queuer.addDelay(1.2);
                robot.dropWrist();
                robot.queuer.addDelay(0.8);
                robot.drop();
                robot.queuer.addDelay(2.5);
                robot.followTrajSeq(dropAndPark[pos]);
                robot.queuer.addDelay(1.0);
                robot.resetAuto();
                robot.queuer.addDelay(3.5);
                robot.resetLift();
                robot.queuer.setFirstLoop(false);
                robot.update();
            }
            robot.stop();
            stop();
    }
}
