package org.firstinspires.ftc.teamcode.Robots;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class IDRobot extends BasicRobot{
    Claw claw;
    CVMaster cv;
    Flip flip;
    Follower follower;
    TelescopicArm arm;
    Twist twist;
    public IDRobot(LinearOpMode opMode, boolean p_isTeleop) {
        super(opMode, p_isTeleop);
        arm = new TelescopicArm();
        claw = new Claw();
        cv = new CVMaster();
        flip = new Flip();
        follower = new Follower(op.hardwareMap);
        twist = new Twist();
    }
    public void setArm(TelescopicArm.ArmStates targ, boolean p_async){
        if(queuer.queue(p_async, targ.getState()))
            arm.goTo(targ);
    }
    public void setClaw(Claw.ClawStates targ, boolean p_async){
        if(queuer.queue(p_async, targ.getState()))
            claw.goTo(targ);
    }
    public void setFlip(Flip.FlipStates targ, boolean p_async){
        if(queuer.queue(p_async, targ.getState()))
            flip.flipTo(targ);
    }
    public void setTwist(Twist.TwistStates targ, boolean p_async){
        if(queuer.queue(p_async, targ.getState()))
            twist.twistTo(targ);
    }
    public void followPath(PathChain path){
        followPath(path, false);
    }
    public void followPath(PathChain path, boolean p_asynchronous){
        if(queuer.queue(p_asynchronous, !follower.isBusy()&&!queuer.isNextExecuted())){
            if(!queuer.isExecuted())
                follower.followPath(path);
        }
    }
    public void update(){
        super.update();
        arm.update();
        claw.update();
        flip.update();
        follower.update();
        arm.update();
        twist.update();
    }
    public void teleOp(){
        boolean isY = gampad.readGamepad(op.gamepad1.y, "gamepad1_y","high basket");
        boolean isB = gampad.readGamepad(op.gamepad1.b, "gamepad1_b", "specimen grab");
        boolean isA = gampad.readGamepad(op.gamepad1.a, "gamepad1_a", "retract slide(flat if from drop, vert if from grab)");
        boolean isX = gampad.readGamepad(op.gamepad1.x, "gamepad1_x", "specimen drop");
        boolean isRB = gampad.readGamepad(op.gamepad1.right_bumper, "gamepad1_right_bumper","down to grab /close claw/open claw");
        boolean isLB = gampad.readGamepad(op.gamepad1.left_bumper, "gamepad1_left_bumper","ground_hover");
        boolean isRD = gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right","auto grab");
        follower.setTeleOpMovementVectors(op.gamepad1.left_stick_y,op.gamepad1.left_stick_x,op.gamepad1.right_stick_x);
        double extend = op.gamepad1.right_trigger-op.gamepad1.left_trigger, rotate = op.gamepad2.right_trigger-op.gamepad2.left_trigger;
        if(abs(extend)>.1 || abs(rotate)>.1)
            arm.manualGoTo(arm.getTargetExt()+extend, arm.getTargetRot()+rotate);
        if(isY){
            arm.goTo(TelescopicArm.ArmStates.HIGH_BUCKET);
            flip.flipTo(Flip.FlipStates.BASKET);
            twist.twistTo(Twist.TwistStates.PARALLEL);
        }
        if(isB){
            arm.goTo(TelescopicArm.ArmStates.SPECIMEN_GRAB);
            flip.flipTo(Flip.FlipStates.SPECIMEN_GRAB);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            claw.goTo(Claw.ClawStates.OPEN);
        }
        if(isA){
            arm.goTo(TelescopicArm.ArmStates.RETRACTED);
            flip.flipTo(Flip.FlipStates.RESET);
            twist.twistTo(Twist.TwistStates.PARALLEL);
        }
        if(isX){
            arm.goTo(TelescopicArm.ArmStates.HIGH_SPECIMEN);
            flip.flipTo(Flip.FlipStates.SPECIMEN);
        }
        if(isRB){
            if(TelescopicArm.ArmStates.HOVER.getState()){
                arm.goTo(TelescopicArm.ArmStates.INTAKE);
                flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
            }
            else if(Claw.ClawStates.OPEN.getState()){
                claw.goTo(Claw.ClawStates.CLOSED);
            }
            else{
                claw.goTo(Claw.ClawStates.OPEN);
            }
        }
        if(isLB){
            arm.goTo(TelescopicArm.ArmStates.HOVER);
            flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
        }
    }
}
