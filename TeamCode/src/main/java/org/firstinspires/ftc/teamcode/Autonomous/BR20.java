package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.voltage;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.imuMultiply;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class BR20 {
    boolean logi=false, isRight;
    LinearOpMode op;
    BradBot robot;
    int bark = 0, delaySec =0, barg=0;
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] intake = new TrajectorySequence[3];
    TrajectorySequence[] backToStack = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence[] drop = new TrajectorySequence[3];
    TrajectorySequence[] park = new TrajectorySequence[3];
    TrajectorySequence[] opark = new TrajectorySequence[3];

    public BR20(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(17, 61, Math.toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);
        imuMultiply = 1.039 + .002*(robot.getVoltage()-12.5);
        spikey[0] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(22,38,toRadians(90)))
                .build();

        spikey[1] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(13, 32, toRadians(90)))
                .build();

        spikey[2] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(10, 37, toRadians(0)), toRadians(180))
                .build();

        if (!isLogi) {
            droppy[0] = robot.roadrun.trajectorySequenceBuilder(spikey[0].end())
                    .lineToLinearHeading(new Pose2d(22,53,toRadians(180)))
                    .lineToLinearHeading(new Pose2d(48.2, 42, toRadians(180))).build();

            droppy[1] = robot.roadrun.trajectorySequenceBuilder(spikey[1].end())
                    .lineToLinearHeading(new Pose2d(13,50, toRadians(90)))
                    .lineToLinearHeading(new Pose2d(48.2, 35.25, toRadians(180))).build();

            droppy[2] = robot.roadrun.trajectorySequenceBuilder(spikey[2].end())
                    .lineToLinearHeading(new Pose2d(30, 35, toRadians(180)))
                    .lineToLinearHeading(new Pose2d(48.2, 30, toRadians(180))).build();

        } else{
        }
        park[0] = robot.roadrun.trajectorySequenceBuilder(droppy[0].end())
                .lineTo(new Vector2d(droppy[0].end().getX()-3, droppy[0].end().getY()))
                .lineToLinearHeading(new Pose2d(44,57, toRadians(180)))
                .build();
        park[1] = robot.roadrun.trajectorySequenceBuilder(droppy[1].end())
                .lineTo(new Vector2d(droppy[1].end().getX()-3, droppy[1].end().getY()))
                .lineToLinearHeading(new Pose2d(44,57, toRadians(180)))
                        .build();
        park[2] = robot.roadrun.trajectorySequenceBuilder(droppy[2].end())
                .lineTo(new Vector2d(droppy[2].end().getX()-3, droppy[2].end().getY()))
                .lineToLinearHeading(new Pose2d(44,57, toRadians(180)))
                .build();
        opark[0] = robot.roadrun.trajectorySequenceBuilder(droppy[0].end())
                .lineTo(new Vector2d(droppy[0].end().getX()-3, droppy[0].end().getY()))
                .lineToLinearHeading(new Pose2d(44,12, toRadians(180)))
                .build();
        opark[1] = robot.roadrun.trajectorySequenceBuilder(droppy[1].end())
                .lineTo(new Vector2d(droppy[1].end().getX()-3, droppy[1].end().getY()))
                .lineToLinearHeading(new Pose2d(44,12, toRadians(180)))
                .build();
        opark[2] = robot.roadrun.trajectorySequenceBuilder(droppy[2].end())
                .lineTo(new Vector2d(droppy[2].end().getX()-3, droppy[2].end().getY()))
                .lineToLinearHeading(new Pose2d(44,12, toRadians(180)))
                .build();

    /*    robot.dropServo(1);
        robot.dropServo(0);*/
        robot.setRight(true);
        robot.setBlue(true);
        robot.observeSpike();
        robot.hoverArm();
    }
    public void waitForStart(){
        while (!op.isStarted() || op.isStopRequested()) {
            bark = 2- robot.getSpikePos();
            op.telemetry.addData("pixel", bark);
            packet.put("spike", bark);
            op.telemetry.addData("delaySec", delaySec);
            op.telemetry.addData("barg,0=L,1=R", barg);
            if (gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "addSecs")) {
                delaySec++;
            }
            if (gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "minusSecs")) {
                delaySec = min(0, delaySec - 1);
            }
            if (gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "parkRight")) {
                barg = 1;
            }
            if (gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "parkLeft")) {
                barg=0;
            }
            robot.update();
        }
        op.resetRuntime();
        time=0;
    }
    public void purp()
    {
//        bark=2;
        robot.queuer.queue(false, true);
        robot.queuer.addDelay(delaySec);
        robot.queuer.waitForFinish();
        robot.followTrajSeq(spikey[bark]);
    }

    public void pre(){
        robot.queuer.waitForFinish();
        robot.followTrajSeq(droppy[bark]);
        if(bark==0) {
            robot.lowAuto(true);
            robot.yellowAuto(true);
            robot.drop(48);
        }
        else if(bark==1) {
            robot.lowAuto(true);
            robot.yellowAuto(true);
            robot.drop(48);
        }
        else {
            robot.lowAuto(false);
            robot.yellowAuto(false);
            robot.drop(48);
        }
    }

    public void park(){
        if(barg == 0){
            robot.followTrajSeq(park[bark]);
        }
        else{
            robot.followTrajSeq(opark[bark]);
        }
        robot.queuer.addDelay(.5);
        robot.resetAuto();
        robot.queuer.addDelay(1);
        robot.queuer.waitForFinish();
        robot.queuer.queue(false, true);
    }

    public void update(){
        robot.update();
        robot.queuer.setFirstLoop(false);
    }

    public boolean isAutDone(){
        return !robot.queuer.isFullfilled()&&time<29.8;
    }
}
