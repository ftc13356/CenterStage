package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RFAprilCam.CONST;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RFAprilCam.Y_OFFSET;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.gampad;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.voltage;
import static org.firstinspires.ftc.teamcode.Robots.BradBot.intakeFInishTIme;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.funnyIMUOffset;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.imuMultiply;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Config
public class RL23 {
    boolean logi=false, isRight,ultras = true, check = true, everChecked = false, intakey = false ;
    LinearOpMode op;
    BradBot robot;
    int bark = 0, delaySec =0;
    boolean lastCycle = false;
    boolean joever = false;
    boolean parky = false;
    TrajectorySequence altPark, altBack;
    double travelTime =4.5, lingerTime = 3;
    public static int overBark =0;

    double[][] ranges = {{0,0},{0,0},{0,0},{0,0},{0,0}};
    TrajectorySequence[] spikey = new TrajectorySequence[3];
    TrajectorySequence[] intake = new TrajectorySequence[3];
    TrajectorySequence[] backToStack = new TrajectorySequence[3];
    TrajectorySequence[] droppy = new TrajectorySequence[3];
    TrajectorySequence[] drop = new TrajectorySequence[3];
    TrajectorySequence[] park= new TrajectorySequence[3], parkLeft= new TrajectorySequence[3];
    int currentSection =0, currentRange = 0;





    public RL23(LinearOpMode op, boolean isLogi){
        logi = isLogi;
        this.op=op;
        robot = new BradBot(op, false,isLogi);
        Pose2d startPose = new Pose2d(-40.5,-61.5,toRadians(-90));
        robot.roadrun.setPoseEstimate(startPose);
        imuMultiply = 1.0132;
        LATERAL_MULTIPLIER = 1.2;
        lastCycle = false;
        joever = false;
        parky = false;
        spikey[0] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-46, -34.25, toRadians(-90)), toRadians(90))
                .addTemporalMarker(robot::done)
                .build();

        spikey[1] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-40,-34,toRadians(-90)))
                .addTemporalMarker(robot::done)
                .build();

        spikey[2] = robot.roadrun
                .trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-34,-38, toRadians(-180)), toRadians(0))
                .build();
        intake[0] = robot.roadrun
                .trajectorySequenceBuilder(spikey[0].end())
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-49, -45.25, toRadians(220)), toRadians(-160))
                .lineToLinearHeading(new Pose2d(-56.7, -35.25, toRadians(180)))
                .addTemporalMarker(robot::stopAllMotors)
                .addTemporalMarker(robot::startIMU)
                .build();
        intake[1] = robot.roadrun
                .trajectorySequenceBuilder(spikey[1].end())
                .lineToLinearHeading(new Pose2d(-56.7,-35.25, toRadians(-180)))
                .addTemporalMarker(robot::stopAllMotors)
                .addTemporalMarker(robot::startIMU)
                .build();
        intake[2] = robot.roadrun
                .trajectorySequenceBuilder(spikey[2].end())
                .lineToLinearHeading(new Pose2d(-56.7,-35.25, toRadians(-180)))
                .addTemporalMarker(robot::stopAllMotors)
                .addTemporalMarker(robot::startIMU)
                .build();


        if (!isLogi) {
            droppy[0] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(intake[0].end())
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(-40,-59.5),toRadians(0))
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(23,5,15))
                            .splineToConstantHeading(new Vector2d(-35,-59.5),toRadians(0))
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                            .splineToConstantHeading(new Vector2d(5,-59.5),toRadians(0))
                            .splineToConstantHeading(new Vector2d(40,-30.75),toRadians(0))
                            .splineToConstantHeading(new Vector2d(46.7,-30.75),toRadians(0))
                            .build();


            droppy[1] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(intake[1].end())
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(-40,-59.5),toRadians(0))
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(23,5,15))
                            .splineToConstantHeading(new Vector2d(-35,-59.5),toRadians(0))
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                            .splineToConstantHeading(new Vector2d(5,-59.5),toRadians(0))
                            .splineToConstantHeading(new Vector2d(40,-37.75),toRadians(0))
                            .splineToConstantHeading(new Vector2d(46.4,-37.75),toRadians(0))
                            .build();

            droppy[2] =
                    robot
                            .roadrun
                            .trajectorySequenceBuilder(intake[2].end())
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(-35,-59.5),toRadians(0))
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10,5,15))
                            .splineToConstantHeading(new Vector2d(-29,-59.5),toRadians(0))
                            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                            .splineToConstantHeading(new Vector2d(5,-59.5),toRadians(0))
                            .splineToConstantHeading(new Vector2d(40,-39.5),toRadians(0))
                            .splineToConstantHeading(new Vector2d(46.5,-39.5),toRadians(0))
                            .build();

        } else{


        }

        backToStack[0] = robot.roadrun
                .trajectorySequenceBuilder(new Pose2d(droppy[0].end().vec(),toRadians(180)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25,-58),toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(27,5,15))
                .splineToConstantHeading(new Vector2d(20,-58),toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                .splineToConstantHeading(new Vector2d(-20,-58),toRadians(180))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-24,-58),toRadians(180))
                .splineToConstantHeading(new Vector2d(-56.2,-35.25),toRadians(180))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(42))
                .build();
        drop[0] = robot.roadrun.trajectorySequenceBuilder(backToStack[0].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40,-57.5),toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(23,5,15))
                .splineToConstantHeading(new Vector2d(-35,-57.5),toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                .splineToConstantHeading(new Vector2d(5,-57.5),toRadians(0))
                .splineToConstantHeading(new Vector2d(42,-37.25),toRadians(0))
                .splineToConstantHeading(new Vector2d(47.7,-37.25),toRadians(0))
                .build();
        backToStack[1] = robot.roadrun
                .trajectorySequenceBuilder(new Pose2d(droppy[1].end().vec(), toRadians(180)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25,-58),toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25,5,15))
                .splineToConstantHeading(new Vector2d(20,-58),toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                .splineToConstantHeading(new Vector2d(-20,-58.5),toRadians(180))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-24,-58.5),toRadians(180))
                .splineToConstantHeading(new Vector2d(-57,-35.25),toRadians(180))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(42))
                .build();
        backToStack[2] = robot.roadrun
                .trajectorySequenceBuilder(new Pose2d(droppy[2].end().vec(),toRadians(180)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25,-58),toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25,5,15))
                .splineToConstantHeading(new Vector2d(20,-58),toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                .splineToConstantHeading(new Vector2d(-20,-58),toRadians(180))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-24,-58),toRadians(180))
                .splineToConstantHeading(new Vector2d(-56.3,-35.25),toRadians(180))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(42))
                .build();
//k is bad
        drop[1] = robot.roadrun.trajectorySequenceBuilder(backToStack[1].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40,-56.5),toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(23,5,15))
                .splineToConstantHeading(new Vector2d(-35,-56.5),toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                .splineToConstantHeading(new Vector2d(5,-56.5),toRadians(0))
                .splineToConstantHeading(new Vector2d(42,-35.25),toRadians(0))
                .splineToConstantHeading(new Vector2d(47.5,-35.25),toRadians(0))
                .build();
        drop[2] = robot.roadrun.trajectorySequenceBuilder(backToStack[2].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, -56.5), toRadians(-15))
                .splineToConstantHeading(new Vector2d(-20, -55.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(6, -56.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(47, -37), toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
    park[1] = robot.roadrun.trajectorySequenceBuilder(drop[1].end())
            .lineToLinearHeading(new Pose2d(43.8,-38, toRadians(-180)))
            .lineToLinearHeading(new Pose2d(45, -58,toRadians(-180)))
            .build();
        altPark = robot.roadrun.trajectorySequenceBuilder(backToStack[1].end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40,-58.5),toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(24,5,15))
                .splineToConstantHeading(new Vector2d(-35,-58.5),toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                .splineToConstantHeading(new Vector2d(5,-58.5),toRadians(0))
                .splineToConstantHeading(new Vector2d(45,-58),toRadians(0))
                .build();
        park[0] = robot.roadrun.trajectorySequenceBuilder(drop[1].end())
                .lineToLinearHeading(new Pose2d(43.8,-38.25, toRadians(-180)))
                .lineToLinearHeading(new Pose2d(45, -58,toRadians(-180)))
                .build();
        parkLeft[1] = robot.roadrun.trajectorySequenceBuilder(drop[1].end())
                .lineToLinearHeading(new Pose2d(43.8,-38, toRadians(-180)))
                .lineToLinearHeading(new Pose2d(45, -11,toRadians(-180)))
                .build();
        altBack = robot.roadrun
                .trajectorySequenceBuilder(new Pose2d(droppy[1].end().vec(), toRadians(180)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(25,-54),toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25,5,15))
                .splineToConstantHeading(new Vector2d(20,-54),toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80,5,15))
                .splineToConstantHeading(new Vector2d(-20,-54),toRadians(180))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-24,-52),toRadians(180))
                .splineToConstantHeading(new Vector2d(-57,-33.25),toRadians(180))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(42))
                .build();

//
//
//
//        parky[0] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[0].end())
//                .lineToLinearHeading(new Pose2d(43.3,-29,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(55,-60,toRadians(-180)))
//                .build();
//
//        parky[1] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[1].end())
//                .lineToLinearHeading(new Pose2d(43.3,-35.5,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(55,-60,toRadians(-180)))
//                .build();
//
//        parky[2] = robot.roadrun
//                .trajectorySequenceBuilder(droppy[2].end())
//                .lineToLinearHeading(new Pose2d(43.3,-41.5,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(50,-60,toRadians(-180)))
//                .lineToLinearHeading(new Pose2d(55,-60,toRadians(-180)))
//                .build();/

        robot.dropServo(1);
        robot.dropServo(0);
        robot.setRight(false);
        robot.setBlue(false);
        robot.observeSpike();
        robot.hoverArm();
    }
    public void waitForStart(){
        Y_OFFSET = 1.6;
        CONST = .005;
        funnyIMUOffset=0.4;
        while (!op.isStarted() || op.isStopRequested()) {
            bark = 2-robot.getSpikePos();
            op.telemetry.addData("pixel", bark);
            packet.put("spike", bark);
            op.telemetry.addData("delaySec", delaySec);
            op.telemetry.addData("isRight", isRight);
            boolean up = gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "addSecs")
                    ,down = gampad.readGamepad(op.gamepad1.dpad_down, "gamepad1_dpad_down", "minusSecs")
                    , right = gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "parkRight"),
                    left = gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "parkLeft"),
                    a = op.gamepad1.a,
                    b = op.gamepad1.b;
            if(a&&up){
                currentRange ++;
                if(currentRange>ranges.length){
                    currentRange =0;
                }
            }
            else if(a&&down){
                currentRange--;
                if(currentRange<0){
                    currentRange = ranges.length-1;
                }
            }
            else if(a && left){
                currentSection--;
                if(currentSection<0){
                    currentSection = 1;
                }
            }
            else if(a&&right){
                currentSection ++;
                if(currentSection>1){
                    currentSection=0;
                }
            }
            else if(b&&up){
                ranges[currentRange][currentSection]++;
            }
            else if(b&&down){
                if(ranges[currentRange][currentSection]>0){
                    ranges[currentRange][currentSection]--;
                }
            }
            else if(b&&right){
                ranges[currentRange][currentSection]+=5;
            }
            else if(b&&left){
                if(ranges[currentRange][currentSection]>4){
                    ranges[currentRange][currentSection]-=5;
                }
            }
            else if (right) {
                isRight = true;
            }
            else if (left) {
                isRight = false;
            }
            String stringify = "";
            for(double[] i : ranges){
                stringify += "["+i[0]+","+i[1]+"]";
            }
            op.telemetry.addData("ranges", stringify);
            packet.put("ranges", stringify);
            robot.update();
        }
        op.resetRuntime();

//        bark=2;
        time=0;
    }
    public void purp()
    {
//        if(bark==0){
//            funnyIMUOffset = 2.3; 3.5
//        }
//        if(bark==1){
//            funnyIMUOffset = 3.5; 3.7
//        }
//        if (bark==2){
//            funnyIMUOffset = 4.4; 3.5
//        }
        robot.queuer.queue(false, true);
        robot.followTrajSeq(spikey[bark]);
        robot.queuer.addDelay(0.0);
    }

    public void intake(int height){
        robot.followTrajSeq(intake[bark]);
        robot.resetAuto();
        if (bark == 0) {
            robot.queuer.addDelay(1.5);
        }
        robot.intakeAuto(height);
    }
    public void cycleIntake(int height){

        robot.followTrajSeq(backToStack[bark]);
        robot.intakeAuto(height);
        robot.queuer.addDelay(0.6);
        robot.resetAuto();
    }
    public void cycleIntake2(int height){
        robot.followTrajSeq(backToStack[1]);
        lastCycle = robot.triggered();
        robot.intakeAuto(height);
        robot.queuer.addDelay(0.6);
        robot.resetAuto();
    }
    public void cycleDrop(int i){
        if(i==1&&robot.triggered()){
            intakey=true;
        }
        robot.queuer.waitForFinish();
        double delTime = 0;
        double arriveTime = intakeFInishTIme+travelTime;
        double leaveTime = arriveTime+lingerTime;
        for(var j : ranges){
            if(arriveTime>j[0]&&arriveTime<j[1])
                delTime = j[1]-arriveTime;
            if(leaveTime>j[0]&&leaveTime<j[1]){
                delTime = max(j[1]-arriveTime,delTime);
            }
        }
        arriveTime = arriveTime+delTime;
        LOGGER.log("arriveTIme" + arriveTime);
        if(arriveTime>=29.75&& !robot.queuer.isNextExecuted()){
            drop[i] = altPark;
            delTime=0;
        }
        robot.queuer.queue(false,true);
        robot.queuer.addDelay(delTime);
        robot.followTrajSeq(drop[i]);
        robot.queuer.addDelay(0.5);
        robot.grabAuto();
        robot.lowAuto(false);
        robot.drop(46);
    }
    public void pre(){
        robot.queuer.waitForFinish();
        double delTime = 0;
        double arriveTime = intakeFInishTIme+travelTime;
        double leaveTime = arriveTime+lingerTime;
        for(var j : ranges){
            if(arriveTime>j[0]&&arriveTime<j[1])
                delTime = j[1]-arriveTime;
            if(leaveTime>j[0]&&leaveTime<j[1]){
                delTime = max(j[1]-arriveTime,delTime);
            }
        }
        arriveTime = arriveTime+delTime;
        LOGGER.log("arriveTIme" + arriveTime);
        if(arriveTime>=29.75&& !robot.queuer.isNextExecuted()){
            droppy[bark] = altPark;
            delTime=0;
        }
        robot.queuer.queue(false,true);
        robot.queuer.addDelay(delTime);
        robot.followTrajSeq(droppy[bark]);
        robot.queuer.addDelay(0.5);
        robot.grabAuto();
    if (bark == 2) {
        robot.lowAuto(true);
        robot.yellowAuto(true);
        robot.drop(45.3);

    }else if(bark==1){
        robot.lowAuto(false);
        robot.yellowAuto(false);
        robot.drop(45.3);
    }
        else{
        robot.lowAuto(false);
        robot.yellowAuto(false);
        robot.drop(45.4);
    }
//        robot.changeIMUInterval();
    }

    public void park(){
        if(currentPose.vec().distTo(park[0].end().vec())>1 || robot.roadrun.isBusy())
            robot.followTrajSeq(park[0]);
        else
            robot.queuer.queue(false, true);
        robot.queuer.addDelay(.4);
        robot.resetAuto();
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

    public void loop(){
        if ((time<21||lastCycle)&&!joever) {
            purp();
            intake(6);
            pre();
            cycleIntake(4);
            cycleDrop(0);
            if(!drop[0].equals(altPark)) {
                cycleIntake2(2);
                cycleDrop(1);
            }
            else{
                robot.queuer.reset();
                joever = true;
            }
        }
        else if(!joever){
            joever = true;
        }
        if(joever && !lastCycle ){
            purp();
            intake(6);
            pre();
            cycleIntake(4);
            cycleDrop(0);
            if(!drop[0].equals(altPark)) {
                cycleIntake2(2);
                cycleDrop(1);
            }
            else{
                robot.queuer.reset();
                joever = true;
                lastCycle
                        = true;
            }
        }
        else if(joever && lastCycle && !parky){
            robot.queuer.reset();
            parky=true;
        }

        park();
        update();
    }


}
