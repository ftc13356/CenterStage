package org.firstinspires.ftc.teamcode.Robots;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.Arrays;

public class IDRobot extends BasicRobot {
    Claw claw;
    CVMaster cv;
    Flip flip;
    public Follower follower;
    TelescopicArm arm;
    Twist twist;
    boolean isAutoGrab = false, targeted = false;
    double lastReadTime;

    public IDRobot(LinearOpMode opMode, boolean p_isTeleop) {
        super(opMode, p_isTeleop);
        arm = new TelescopicArm();
        claw = new Claw();
        cv = new CVMaster();
        flip = new Flip();
        follower = new Follower(op.hardwareMap);
        twist = new Twist();
        isAutoGrab = false;
        targeted = false;
        lastReadTime = -100;
    }

    public void autoReset(boolean p_async){
        if(queuer.queue(p_async, TelescopicArm.ArmStates.RETRACTED.getState())) {
            arm.goTo(TelescopicArm.ArmStates.RETRACTED);
            claw.goTo(Claw.ClawStates.OPEN);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            flip.flipTo(Flip.FlipStates.RESET);
        }
    }

    public void setArm(TelescopicArm.ArmStates targ, boolean p_async) {
        if (queuer.queue(p_async, targ.getState()) && !queuer.isExecuted() && !queuer.isFirstLoop())
            arm.goTo(targ);
    }

    public void setClaw(Claw.ClawStates targ, boolean p_async) {
        if (queuer.queue(p_async, targ.getState()) && !queuer.isExecuted() && !queuer.isFirstLoop())
            claw.goTo(targ);
    }

    public void setFlip(Flip.FlipStates targ, boolean p_async) {
        if (queuer.queue(p_async, targ.getState()) && !queuer.isExecuted() && !queuer.isFirstLoop())
            flip.flipTo(targ);
    }

    public void setTwist(Twist.TwistStates targ, boolean p_async) {
        if (queuer.queue(p_async, targ.getState()))
            twist.twistTo(targ);
    }

    public void followPath(PathChain path) {
        followPath(path, false);
    }

    public void followPath(PathChain path, boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, !follower.isBusy() && !queuer.isNextExecuted())) {
            if (!queuer.isExecuted())
                follower.followPath(path);
        }
    }
    public void followPath(Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, boolean is_Dynamic) {
        if (queuer.queue(p_asynchronous, !follower.isBusy() && !queuer.isNextExecuted())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .build();
                follower.followPath(path2);
            }
        }
    }
    public void lowerToGround(boolean p_async){
        if (queuer.queue(p_async, TelescopicArm.ArmStates.INTAKE.getState()))
            arm.lowerToIntake();
    }

    public void placeSample(Point score){
        followPath(score, 0, -Math.PI/4,false,true);
        setArm(TelescopicArm.ArmStates.HIGH_BUCKET, true);
        setTwist(Twist.TwistStates.PARALLEL, true);
        setFlip(Flip.FlipStates.BUCKET, true);
        setClaw(Claw.ClawStates.OPEN, false);
    }


    public void autoGrab() {
        if (queuer.queue(false, !isAutoGrab && TelescopicArm.ArmStates.INTAKE.getState())) {
            if (TelescopicArm.ArmStates.HOVER.getState()) {
                if (!isAutoGrab) {
                    cv.resetCenter();
                    targeted = false;
                }
                isAutoGrab = true;
                follower.stopTeleopDrive();
                double[] relCent = cv.getCenter();
                if (!Arrays.equals(relCent, new double[]{0, 0, 0})) {
                    targeted = true;
                    relCent[0] = relCent[2] * Math.sin(arm.getRot()) + relCent[0] * Math.cos(arm.getRot());
                    if (relCent[0] * relCent[0] + relCent[1] * relCent[1] < 1) {
                        arm.lowerToIntake();
                        flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
                        isAutoGrab = false;
                        targeted = false;
                    } else {
                        Vector2d relVect = new Vector2d(0, -relCent[1]).rotated(-follower.getPose().getHeading());
                        Pose pos = follower.getPose();
                        pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                        follower.holdPoint(new BezierPoint(new Point(pos)), pos.getHeading());
                        double newExt = arm.getExt() + relCent[0];
                        arm.goTo(newExt, atan2(6, newExt));
                    }
                } else if (!targeted) {
                    arm.manualGoTo(0.5, 0);
                }
            } else {
                arm.goTo(TelescopicArm.ArmStates.HOVER);
                flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
            }
        }
    }

    public void update() {
        super.update();
        arm.update();
        claw.update();
        flip.update();
        follower.update();
        twist.update();
    }

    public void teleOp() {
        boolean isY = gampad.readGamepad(op.gamepad1.y, "gamepad1_y", "high basket");
        boolean isB = gampad.readGamepad(op.gamepad1.b, "gamepad1_b", "specimen grab");
        boolean isA = gampad.readGamepad(op.gamepad1.a, "gamepad1_a", "retract slide(flat if from drop, vert if from grab)");
        boolean isX = gampad.readGamepad(op.gamepad1.x, "gamepad1_x", "specimen drop");
        boolean isRB = gampad.readGamepad(op.gamepad1.right_bumper, "gamepad1_right_bumper", "down to grab /close claw/open claw");
        boolean isLB = gampad.readGamepad(op.gamepad1.left_bumper, "gamepad1_left_bumper", "ground_hover");
        boolean isRD = gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "auto grab red");
        boolean isLD = gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "auto grab yellow");
        boolean isUD = gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "auto grab blue");


        boolean isDD = op.gamepad1.dpad_down;

        if (follower.isTeleDrive() || (abs(op.gamepad1.left_stick_y) > 0.001 || abs(op.gamepad1.left_stick_x) > 0.001 || abs(op.gamepad1.right_stick_x) > 0.001)) {
            if (!follower.isTeleDrive())
                follower.startTeleopDrive();
            follower.setTeleOpMovementVectors(-op.gamepad1.left_stick_y*.3, -op.gamepad1.left_stick_x*.1, -op.gamepad1.right_stick_x*.3);
            isAutoGrab = false;
        }
        double extend = op.gamepad1.right_trigger - op.gamepad1.left_trigger, rotate = op.gamepad2.right_trigger - op.gamepad2.left_trigger;
        if (abs(extend) > .1 || abs(rotate) > .1) {
            arm.manualGoTo(extend, rotate);
            isAutoGrab = false;
        }
        if(isDD && isY){
            arm.goTo(TelescopicArm.ArmStates.LOW_BUCKET);
            flip.flipTo(Flip.FlipStates.BUCKET);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            isAutoGrab = false;
        }
        else if (isY) {
            arm.goTo(TelescopicArm.ArmStates.HIGH_BUCKET);
            flip.flipTo(Flip.FlipStates.BUCKET);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            isAutoGrab = false;
        }
        if (isB) {
            arm.goTo(TelescopicArm.ArmStates.SPECIMEN_GRAB);
            flip.flipTo(Flip.FlipStates.SPECIMEN_GRAB);
            twist.twistTo(Twist.TwistStates.PERPENDICULAR);
            claw.goTo(Claw.ClawStates.OPEN);
            isAutoGrab = false;
        }
        if (isA) {
            arm.goTo(TelescopicArm.ArmStates.RETRACTED);
            flip.flipTo(Flip.FlipStates.SPECIMEN);
            isAutoGrab = false;
        }
        if(isDD && isX){
            arm.goTo(TelescopicArm.ArmStates.LOW_SPECIMEN);
            flip.flipTo(Flip.FlipStates.SPECIMEN);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            isAutoGrab = false;
        }
        else if (isX) {
            arm.goTo(TelescopicArm.ArmStates.HIGH_SPECIMEN);
            flip.flipTo(Flip.FlipStates.SPECIMEN);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            isAutoGrab = false;
        }
        if(time-lastReadTime > 1 && !isAutoGrab) {
            double[] relCen = cv.getCenter().clone();
            if (!Arrays.equals(relCen, new double[]{0, 0, 0})) {
                targeted = true;
                relCen[0] = relCen[2] * Math.sin(arm.getRot() * PI / 180) + relCen[0] * Math.cos(arm.getRot() * PI / 180) - 2.5;
                packet.put("relCent0", relCen[0]);
                packet.put("relCent1", relCen[1]);
            }
            lastReadTime = time;
        }
        if (isRD || isLD || isUD || isAutoGrab) {
            if(isRD)
                cv.swapRed();
            else if(isLD)
                cv.swapBlue();
            else if (isUD)
                cv.swapYellow();
            if(isRD || isLD || isUD) {
                cv.resetCenter();
                flip.flipTo(Flip.FlipStates.RESET);
            }
            if (TelescopicArm.ArmStates.HOVER.getState() || TelescopicArm.ArmStates.LOW_SPECIMEN.getState()) {
                if (!isAutoGrab) {
                    cv.resetCenter();
                    targeted = false;
                }
                isAutoGrab = true;
                if(follower.isTeleDrive())
                    follower.stopTeleopDrive();
                double[] relCent = cv.getCenter().clone();
                if (!Arrays.equals(relCent, new double[]{0, 0, 0, 0})) {
                    targeted = true;
                    cv.resetCenter();
                    relCent[0] = relCent[2]*Math.sin(arm.getRot()*PI/180)+relCent[0]*Math.cos(arm.getRot()*PI/180)-2.3;
                    packet.put("relCent0", relCent[0]);
                    packet.put("relCent1", relCent[1]);
                    if (relCent[0] * relCent[0] + relCent[1] * relCent[1] < 0.5 && abs(arm.getVel())<3) {
                        isRB = true;
                        isAutoGrab = false;
                        targeted = false;
                    } else {
                        Vector2d relVect = new Vector2d(0, -relCent[1]).rotated(-follower.getPose().getHeading());
                        Pose pos = follower.getPose();
                        pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                        double head = pos.getHeading();
                        if(follower.getClosestPose()!=null)
                            head = follower.getClosestPose().getHeading();
                        follower.holdPoint(new BezierPoint(new Point(pos)), head);
                        double newExt = arm.getExt() + relCent[0] - arm.getVel()*.1;
                        arm.goToResetManual(newExt, Math.atan2(6, newExt+7)*180/PI);
                        twist.twistToAng(relCent[3]);
                        packet.put("newExt", newExt);
                        packet.put("relVect", relVect);
                        packet.put("relAng", relCent[3]);
//                        isAutoGrab = false;
                    }
                } else if (!targeted) {
                    arm.manualGoTo(1, 0);
                }
            } else {
                arm.goTo(TelescopicArm.ArmStates.HOVER);
                flip.flipTo(Flip.FlipStates.RESET);
            }
        }
        if (isLB) {
            arm.goTo(TelescopicArm.ArmStates.HOVER);
            flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            isAutoGrab = false;
        }
        if (isRB) {
            if (TelescopicArm.ArmStates.HOVER.getState()) {
                if(!Flip.FlipStates.SUBMERSIBLE.getState())
                    flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
                else
                    arm.lowerToIntake();
            } else if (Claw.ClawStates.OPEN.getState()) {
                claw.goTo(Claw.ClawStates.CLOSED);
            } else {
                claw.goTo(Claw.ClawStates.OPEN);
            }
            isAutoGrab = false;
        }
        update();
    }
}