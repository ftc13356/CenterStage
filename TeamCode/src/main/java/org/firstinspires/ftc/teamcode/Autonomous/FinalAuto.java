package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "FinalAuto")
public class FinalAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        ElapsedTime runtime = new ElapsedTime();

        int rings = robot.getRingsAndWaitForStart();
        robot.stopRingDetection();
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
        if(rings!=1&&rings!=4) {
            robot.goToPosition( -64,-1,0, 1);//-25,60
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-60.5,29.8,-2,0.8);//37,4
            robot.shootThreePowerShot();
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
            robot.turnInPlace(0, 1.0);
            //robot.moveAngle(6.5, 49.5, 0.8);
            robot.goToPosition(-53.5+44,32+3,0,0.8);
            robot.turnInPlace(0,0.6);
            //robot.moveAngle(-4.0,0,0.8);
            robot.goToPosition(-53.5+44,32+3-6.9,0,0.8);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            //robot.moveAngle(-23,-59,0.7);
            robot.goToPosition(-53.5+43.5-56.5,32+3-6.5-28,0,0.8);
            robot.openWobbleGoalClaw();
            sleep(250);
            //robot.moveAngle(10,-2,0.7);
            robot.goToPosition(-53.5+43.5-59-2,32+6.5-4-23+10,0,0.8);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(500);
        }
        else if(rings==1) {
            robot.goToPosition(-98,12,0,1);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.goToPosition(-60.5,23,-2.5,0.7);
            robot.shootThreePowerShot();
            robot.turnInPlace(0,1.0);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
            robot.turnInPlace(0, 1.0);
//                robot.moveAngle(4.25, 47.5, 0.8);
            robot.goToPosition(-53.5+44,26,0,0.8);
            robot.turnInPlace(0,1.0);
////                robot.moveAngle(-4.0,0,0.8);
            robot.goToPosition(-53.5+44,19,0,0.8);
            sleep(250);
            robot.closeWobbleGoalClaw();
            sleep(600);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.goToPosition(-88,12,0,0.8);
//                robot.moveAngle(-0.5,-80,0.8);
            robot.openWobbleGoalClaw();
            sleep(250);
//                robot.moveAngle(5,15,0.8);
            robot.goToPosition(-72,17,0,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(500);
        }
        else if(rings==4) {
            robot.goToPosition(-23,0,0,0.8);
//            robot.moveBackward(23, 0.8);
            robot.shootGoalTeleop(120);
            robot.turnInPlace(-4, 1);
            robot.shootHighGoal(3);
//            robot.turnInPlace(0, 1);
            robot.goToPosition(-23,-9.5,0,1);
//            robot.moveLeft(8.5, 1);
            robot.turnInPlace(-2.5, 1);
            robot.goToPosition(-115, -7, 0,0.9);
//            robot.moveBackward(88, 1);
            robot.openWobbleGoalClaw();
//            robot.goToPosition(-20);
            robot.goToPosition(-55,2,0,1);
            robot.goToPosition(-49,2,0,1);
//            robot.goToPosition(-51,2,0,1);
            robot.startIntake();
            robot.startTransfer();
//            robot.moveAngle(8, 50,0.9);
            robot.goToPosition(-41, 1, -5,0.7);
            robot.shootHighGoal(1);
            robot.turnInPlace(0,1);
            robot.goToPosition(-26, 1, 0,1);
            robot.turnInPlace(-5,1);
            robot.shootHighGoal(3);
            robot.goToPosition(-70,1, 0,1);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(200);
        }
        stop();
    }
}
