package org.firstinspires.ftc.teamcode.Robots;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Queue;

@Config
public class IDRobot extends BasicRobot {
    Claw claw;
    CVMaster cv;
    Flip flip;
    public Follower follower;
    ArrayList<Queuer> queuers;
    TelescopicArm arm;
    Twist twist;
    boolean isAutoGrab = false, targeted = false;
    double lastReadTime;
    Point lastTarg = new Point(0,0,1);
    public static double FOR_CONST =2, FOR_MULT = 0.6, SIDE_CONST = 1.5, SIDE_MULT = 1.2, MOVE_INTERVAL = 0.75;
    double lastMoveTime = -100;

    public IDRobot(LinearOpMode opMode, boolean p_isTeleop) {
        super(opMode, p_isTeleop);
        arm = new TelescopicArm();
        claw = new Claw();
        cv = new CVMaster();
        flip = new Flip();
        follower = new Follower(op.hardwareMap);
        queuers = new ArrayList<>();
        twist = new Twist();
        isAutoGrab = false;
        targeted = false;
        lastReadTime = -100;
        lastMoveTime = -100;
        queuers.add(new Queuer());//FLIP_AND_RESET
        queuers.add(new Queuer());//HIGH_BASKET
        queuers.add(new Queuer());//LOWER_INTAKE&GRAB
        queuers.add(new Queuer());//SPECI_RESET
    }

    public void autoReset(boolean p_async) {
        if (queuer.queue(p_async, TelescopicArm.ArmStates.RETRACTED.getState()&&abs(arm.getExt())<2)) {
            if(!queuer.isExecuted()) {
                claw.goTo(Claw.ClawStates.OPEN);
                twist.twistTo(Twist.TwistStates.PARALLEL);
                flip.flipTo(Flip.FlipStates.RESET);
                arm.goTo(TelescopicArm.ArmStates.RETRACTED);
            }
        }
    }

    public void setArm(TelescopicArm.ArmStates targ, boolean p_async) {
        if (queuer.queue(p_async, targ.getState()&&abs(targ.getExtendPos()-arm.getExt())<1) && !queuer.isExecuted() && !queuer.isFirstLoop())
            arm.goTo(targ);
    }

    public void setArm(double extension, double rot, boolean p_async) {
        if (queuer.queue(p_async, abs(arm.getExt() - extension) < 2 && abs(arm.getRot() - rot) < 5) && !queuer.isExecuted() && !queuer.isFirstLoop())
            arm.goTo(extension, rot);
    }

    public void setClaw(Claw.ClawStates targ, boolean p_async) {
        if (queuer.queue(p_async, targ.getState()) && !queuer.isExecuted() && !queuer.isFirstLoop())
            claw.goTo(targ);
    }
    public void setClaw(Claw.ClawStates targ, boolean p_async, Queuer queuer) {
        if (queuer.queue(p_async, targ.getState()) && !queuer.isExecuted() && !queuer.isFirstLoop())
            claw.goTo(targ);
    }

    public void setFlip(Flip.FlipStates targ, boolean p_async) {
        if (queuer.queue(p_async, targ.getState()) && !queuer.isExecuted() && !queuer.isFirstLoop())
            flip.flipTo(targ);
    }

    public void setTwist(Twist.TwistStates targ, boolean p_async, Queuer queuer) {
        if (queuer.queue(p_async, targ.getState()))
            twist.twistTo(targ);
    }
    public void setFlip(Flip.FlipStates targ, boolean p_async, Queuer queuer) {
        if (queuer.queue(p_async, targ.getState()) && !queuer.isExecuted() && !queuer.isFirstLoop())
            flip.flipTo(targ);
    }
    public void setArm(TelescopicArm.ArmStates targ, boolean p_async, Queuer queuer) {
        if (queuer.queue(p_async, targ.getState()&&abs(targ.getExtendPos()-arm.getExt())<1) && !queuer.isExecuted() && !queuer.isFirstLoop())
            arm.goTo(targ);
    }

    public void setTwist(Twist.TwistStates targ, boolean p_async) {
        if (queuer.queue(p_async, targ.getState()))
            twist.twistTo(targ);
    }

    public void setTwist(double rotation, boolean p_async) {
        if (queuer.queue(p_async, true))
            twist.twistTo(rotation);
    }


    public void followPath(PathChain path) {
        followPath(path, false);
    }

    public void followPath(PathChain path, boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted())
                follower.followPath(path);
        }
    }

    public void followPath(Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
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
    public void followPath(Point end, double headingInterp0, double headingInterp1, int headingInterp, boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 ;
                if(headingInterp==0) {
                path2 =follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), end))
                            .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                            .build();
                } else{
                    path2 =follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), end))
                            .setConstantHeadingInterpolation(headingInterp0)
                            .build();
                }
                follower.followPath(path2);
            }
        }
    }
    public void followPath(Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, boolean hodlPoint) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .build();
                follower.followPath(path2, hodlPoint);
            }
        }
    }
    public void followPath(Point mid, Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, boolean holdEnd) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), mid, end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .build();
                follower.followPath(path2,holdEnd);
            }
        }
    }

    public void followPath(Point mid, Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), mid, end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .build();
                follower.followPath(path2);
            }
        }
    }

    public void lowerToGround(boolean p_async) {
        if (queuer.queue(p_async, TelescopicArm.ArmStates.INTAKE.getState()))
            arm.lowerToIntake();
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
                        arm.goTo(newExt, atan2(6, newExt+10));
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

    public void teleOp(boolean isBlue) {
        boolean isY = gampad.readGamepad(op.gamepad1.y, "gamepad1_y", "high basket");
        boolean isB = gampad.readGamepad(op.gamepad1.b, "gamepad1_b", "specimen grab");
        boolean isA = gampad.readGamepad(op.gamepad1.a, "gamepad1_a", "retract slide(flat if from drop, vert if from grab)");
        boolean isX = gampad.readGamepad(op.gamepad1.x, "gamepad1_x", "specimen drop");
        boolean isRB = gampad.readGamepad(op.gamepad1.right_bumper, "gamepad1_right_bumper", "down to grab /close claw/open claw");
        boolean isSuperRB = gampad.readGamepad(op.gamepad2.right_bumper, "gamepad2_right_bumper", "down to grab /close claw/open claw");
        boolean isSuperLB = gampad.readGamepad(op.gamepad2.left_bumper, "gamepad2_left_bumper", "down to grab /close claw/open claw");
        boolean isLB = gampad.readGamepad(op.gamepad1.left_bumper, "gamepad1_left_bumper", "ground_hover");
        boolean isSuperY = gampad.readGamepad(op.gamepad2.y, "gamepad2_y", "down to grab /close claw/open claw");
        boolean isRD = gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "auto grab red");
        boolean isLD = gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "auto grab yellow");
        boolean isUD = gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "auto grab blue");



        boolean isDD = op.gamepad1.dpad_down;
        boolean isDD2 = op.gamepad2.dpad_down;

        double driveConst = 0.7;
        if (TelescopicArm.ArmStates.HIGH_BUCKET.getState() || TelescopicArm.ArmStates.HOVER.getState() || TelescopicArm.ArmStates.SPECIMEN_GRAB.getState()) {
            driveConst = 0.3;
        }

        if (follower.isTeleDrive() || (abs(op.gamepad1.left_stick_y) > 0.001 || abs(op.gamepad1.left_stick_x) > 0.001 || abs(op.gamepad1.right_stick_x) > 0.001)) {
            if (!follower.isTeleDrive()) {
                follower.startTeleopDrive();
                follower.breakFollowing();
            }
            follower.setTeleOpMovementVectors(-op.gamepad1.left_stick_y * driveConst, -op.gamepad1.left_stick_x * .5 * driveConst, -op.gamepad1.right_stick_x * driveConst * .5);
//            isAutoGrab = false;
        }
        double extend = op.gamepad1.right_trigger - op.gamepad1.left_trigger;
        if (abs(extend) > .1) {
            if (!isDD)
                arm.manualGoTo(extend, 0);
            else
                arm.manualGoTo(0, extend);
//            isAutoGrab = false;
        }
        if (isDD && isY) {
            arm.goTo(TelescopicArm.ArmStates.LOW_BUCKET);
            flip.flipTo(Flip.FlipStates.BUCKET);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            isAutoGrab = false;
        } else if (isY || !queuers.get(1).isEmpty()) {
            if (TelescopicArm.ArmStates.HIGH_BUCKET.getState() && queuers.get(1).isEmpty()) {
                flip.flipTo(Flip.FlipStates.BUCKET);
                twist.twistTo(Twist.TwistStates.PERPENDICULAR);
            } else if(queuers.get(0).isEmpty()){
                setArm(TelescopicArm.ArmStates.HIGH_BUCKET,false,queuers.get(1));
                setFlip(Flip.FlipStates.RESET,true,queuers.get(1));
                setTwist(Twist.TwistStates.PERPENDICULAR,true,queuers.get(1));
                setFlip(Flip.FlipStates.BUCKET, false,queuers.get(1));
            }
            isAutoGrab = false;
        }
        if(isB&&isDD){
            arm.goTo(TelescopicArm.ArmStates.SPECIMEN_GRAB);
            flip.flipTo(Flip.FlipStates.SPECIMEN_GRAB);
            twist.twistTo(Twist.TwistStates.PERPENDICULAR);
            isAutoGrab = false;
        }
        else if (isB) {
                arm.goTo(TelescopicArm.ArmStates.SPECIMEN_GRAB);
                flip.flipTo(Flip.FlipStates.SPECIMEN_GRAB);
                twist.twistTo(Twist.TwistStates.SPECIMEN);
                isAutoGrab = false;
        }
        if (isA||!queuers.get(0).isEmpty()||!queuers.get(3).isEmpty()) {
            if(isA){
                for(var i : queuers)
                    i.reset();
            }
            if (!queuers.get(0).isEmpty() || ((TelescopicArm.ArmStates.HIGH_BUCKET.getState() || TelescopicArm.ArmStates.INTAKE.getState()) && !Flip.FlipStates.RESET.getState())) {
                if(queuers.get(0).isEmpty()) {
                    if ((TelescopicArm.ArmStates.INTAKE.getState() || TelescopicArm.ArmStates.HOVER.getState()) && arm.getRot() < 40) {
                        follower.stopTeleopDrive();
                        Vector2d relVect = new Vector2d(-5, 0).rotated(follower.getPose().getHeading());
                        Pose pos = follower.getPose();
                        pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                        follower.holdPoint(new BezierPoint(new Point(pos)), follower.getPose().getHeading());
                    }
                    if (TelescopicArm.ArmStates.HIGH_BUCKET.getState()) {
                        follower.stopTeleopDrive();
                        Vector2d relVect = new Vector2d(5, 0).rotated(follower.getPose().getHeading());
                        Pose pos = follower.getPose();
                        pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                        follower.holdPoint(new BezierPoint(new Point(pos)), follower.getPose().getHeading());
                    }
                }
                    setFlip(Flip.FlipStates.RESET,false, queuers.get(0));
                    setTwist(Twist.TwistStates.PERPENDICULAR,true, queuers.get(0));
                    queuers.get(0).addDelay(0.3);
                    setArm(TelescopicArm.ArmStates.RETRACTED, false, queuers.get(0));
                    isAutoGrab = false;
            }else if (!queuers.get(3).isEmpty()||(TelescopicArm.ArmStates.HIGH_SPECIMEN.getState() && !Claw.ClawStates.OPEN.getState()) ) {
                setFlip(Flip.FlipStates.RETRACT, false, queuers.get(3));
                setClaw(Claw.ClawStates.OPEN, true, queuers.get(3));
                setArm(TelescopicArm.ArmStates.RETRACTED,false, queuers.get(3));
            } else {
                arm.goTo(TelescopicArm.ArmStates.RETRACTED);
                flip.flipTo(Flip.FlipStates.RESET);
                isAutoGrab = false;
            }
        }
        if (isDD && isX) {
            arm.goTo(TelescopicArm.ArmStates.LOW_SPECIMEN);
            flip.flipTo(Flip.FlipStates.SPECIMEN);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            isAutoGrab = false;
        } else if (isX) {
            arm.goTo(TelescopicArm.ArmStates.HIGH_SPECIMEN);
            flip.flipTo(Flip.FlipStates.SPECIMEN);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            isAutoGrab = false;
        }
        if (time - lastReadTime > 1 && !isAutoGrab) {
            double[] relCen = cv.getCenter().clone();
            if (!Arrays.equals(relCen, new double[]{0, 0, 0})) {
                targeted = true;
                relCen[0] = relCen[2] * Math.sin(arm.getRot() * PI / 180) + relCen[0] * Math.cos(arm.getRot() * PI / 180) - 1.5;
                packet.put("relCent0", relCen[0]);
                packet.put("relCent1", relCen[1]);
            }
            lastReadTime = time;
        }
        if (isRD || isUD || isAutoGrab) {
            if (isRD && isBlue)
                cv.swapBlue();
            else if (isRD)
                cv.swapRed();
            else if (isUD)
                cv.swapYellow();
            if (isRD || isLD || isUD) {
                cv.resetCenter();
                flip.flipTo(Flip.FlipStates.RESET);
            }
            if (TelescopicArm.ArmStates.HOVER.getState() || TelescopicArm.ArmStates.LOW_SPECIMEN.getState()) {
                if (!isAutoGrab) {
                    cv.resetCenter();
                    targeted = false;
                }
                isAutoGrab = true;

                double[] relCent = cv.getCenter().clone();
                if (!Arrays.equals(relCent, new double[]{0, 0, 0, 0})) {
                    targeted = true;
                    cv.resetCenter();
                    relCent[0] = (relCent[2] * Math.sin(arm.getRot() * PI / 180) + relCent[0] * Math.cos(arm.getRot() * PI / 180) - FOR_CONST)*FOR_MULT;
                    packet.put("relCent0", relCent[0]);
                    packet.put("relCent1", relCent[1]);
                    if (follower.isTeleDrive())
                        follower.stopTeleopDrive();
                    if (relCent[0] * relCent[0] + relCent[1] * relCent[1] < 200) {
                        if (relCent[0] * relCent[0] + relCent[1] * relCent[1] < 1 && abs(arm.getVel()) + follower.getVelocityMagnitude() < 0.9) {
                            if(!Flip.FlipStates.SUBMERSIBLE.getState()) {
                                flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
                                claw.goTo(Claw.ClawStates.OPEN);
                            }
                            else{
                                arm.lowerToIntake();
                                isAutoGrab = false;
                                targeted = false;
                            }
                        } else if(time - lastMoveTime>MOVE_INTERVAL){
//                            flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
//                            claw.goTo(Claw.ClawStates.OPEN);
                            Vector2d relVect = new Vector2d(0, ((-relCent[1] + Math.signum(-relCent[1])*SIDE_CONST))*SIDE_MULT).rotated(follower.getPose().getHeading());
                            Vector2d relVect2 = new Vector2d(0, (-relCent[1]*SIDE_MULT)).rotated(follower.getPose().getHeading());
                            Pose pos = follower.getPose();
                            pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                            Pose pos2 = follower.getPose();
                            pos2.add(new Pose(relVect2.getX(), relVect2.getY(), 0));

                            double head = follower.getPose().getHeading();
                            if (follower.getCurrentPath() != null) {
                                head = follower.getCurrentPath().getHeadingGoal(1);
                            }
                            Point curTarg = lastTarg;
                            Point newTarg = new Point(pos2);
                            if (curTarg == null) {
                                follower.holdPoint(new BezierPoint(new Point(pos)), head);
                            } else if (curTarg.distanceFrom(newTarg) > 0.8) {
                                follower.holdPoint(new BezierPoint(new Point(pos)), head);
                            }
                            double newExt = arm.getExt() + relCent[0] - arm.getVel() * .1;
                            arm.goToResetManual(newExt, Math.atan2(5, newExt + 8) * 180 / PI);
                            twist.twistToAng(relCent[3]);
                            packet.put("newExt", newExt);
                            packet.put("relVect", relVect);
                            packet.put("relAng", relCent[3]);
                            lastTarg = new Point(pos2);
                            lastMoveTime = time;
                        }
                    }
                } else if (!targeted) {
//                    arm.manualGoTo(1, 0);
                }
            } else {
                arm.goTo(TelescopicArm.ArmStates.HOVER);
                flip.flipTo(Flip.FlipStates.RESET);
                isAutoGrab=true;
            }
        }
        if (op.gamepad1.dpad_down && isLB) {
            twist.iterateTwist(-1);
        } else if (isLB) {
            arm.goTo(TelescopicArm.ArmStates.HOVER);
            flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            isAutoGrab = false;
        }
        if (op.gamepad1.dpad_down && isRB) {
            twist.iterateTwist(1);
        } else if (isRB||!queuers.get(2).isEmpty()) {
            if ((!queuers.get(2).isEmpty() || TelescopicArm.ArmStates.HOVER.getState())&&queuers.get(0).isEmpty()) {
               setFlip(Flip.FlipStates.SUBMERSIBLE,false,queuers.get(2));
               if(queuers.get(2).queue(false, true)){
                   arm.lowerToIntake();
               }
            } else if (TelescopicArm.ArmStates.HIGH_BUCKET.getState() && Claw.ClawStates.CLOSED.getState()) {
                claw.goTo(Claw.ClawStates.OPEN);
            } else if (Claw.ClawStates.OPEN.getState()) {
                claw.goTo(Claw.ClawStates.CLOSED);
            } else {
                claw.goTo(Claw.ClawStates.OPEN);
            }
            isAutoGrab = false;
        }
        if(isDD2&&isSuperRB){
            arm.lowerToIntake();
            flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
            claw.goTo(Claw.ClawStates.OPEN);
//            isAutoGrab = false;
        }
        if(isDD2 && isSuperLB){
            claw.goTo(Claw.ClawStates.CLOSED);
        }

        if(isDD2 && isSuperY){
            arm.goTo(TelescopicArm.ArmStates.HIGH_BUCKET);
            flip.flipTo(Flip.FlipStates.BUCKET);
            twist.twistTo(Twist.TwistStates.PERPENDICULAR);
//            isAutoGrab = false;
        }
        for(var i : queuers){
            if(!i.isEmpty()){
                i.setFirstLoop(false);
            }
            if(!i.isEmpty()&& i.isFullfilled()){
                i.reset();
            }
        }
        update();
    }
}
