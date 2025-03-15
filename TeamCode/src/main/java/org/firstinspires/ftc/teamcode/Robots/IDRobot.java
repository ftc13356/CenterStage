package org.firstinspires.ftc.teamcode.Robots;

import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.CYCLE_OFFSET_X;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.CYCLE_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST12_X;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST12_Y;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST1_X;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST1_Y;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST2_X;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST2_Y;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST3_X;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST3_Y;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST4_X;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST4_Y;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST5_X;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DIST5_Y;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DISTTWO_X;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.DISTTWO_Y;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.H_OFFSET;
import static org.firstinspires.ftc.teamcode.Components.Constants.AutoSpec.RAISE_DELAY;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHBUCKET_EXTEND_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHBUCKET_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.HIGHSPECIMEN_PITCH_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.LOWBUCKET_EXTEND_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.SPECIMENGRAB_EXTEND_POS;
import static org.firstinspires.ftc.teamcode.Components.TelescopicArm.SPECIMENGRAB_PITCH_POS;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.max;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.analysis.function.Max;
import org.firstinspires.ftc.teamcode.Components.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Flip;
import org.firstinspires.ftc.teamcode.Components.HangServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
import org.firstinspires.ftc.teamcode.Components.TelescopicArm;
import org.firstinspires.ftc.teamcode.Components.Twist;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Queue;

@Config
public class IDRobot extends BasicRobot {
    public Claw claw;
    CVMaster cv;
    Flip flip;
    public Follower follower;
    HangServo hang;
    public ArrayList<Queuer> queuers;
    public TelescopicArm arm;
    Twist twist;
    boolean isAutoGrab = false, targeted = false; //sigma
    double lastReadTime, lastStartAUtoGrabTime = -100;
    Point lastTarg = new Point(0, 0, 1);
    public static double FOR_CONST = 3.20, FOR_MULT = 1, SIDE_CONST = 1.7, SIDE_MULT = 1, MOVE_INTERVAL = 0.5, DELAY_TIME = 0.24, DROP_DELAY_TIME = 0.15, MIN_EXT = 6.0, HANGEXT1 = 25.5, HANGROT1 = 70,
            HANGEXT2 = 0, HANGROT2 = 120, HANGEXT3 = 3.5, HANGROT3 = 20, LAG_CONSST = .25, MAX_EXT = 19, RETRACT_CONST = 0, STABLIZE_TIME = 0., DROP_DEL = 0.1;
    double driveConst = .7;
    double lastMoveTime = -100;
    Pose grabPoint = new Pose(0, 0, 0);

    public IDRobot(LinearOpMode opMode, boolean p_isTeleop) {
        super(opMode, p_isTeleop);
        arm = new TelescopicArm();
        claw = new Claw();
        cv = new CVMaster();
        flip = new Flip();
        follower = new Follower(op.hardwareMap);
        hang = new HangServo();
        queuers = new ArrayList<>();
        twist = new Twist();
        isAutoGrab = false;
        targeted = false;
        lastReadTime = -100;
        lastMoveTime = -100;
        for (int i = 0; i < 10; i++) {
            queuers.add(new Queuer());
        }

    }

    public void autoReset(boolean p_async) {
        if (queuer.queue(p_async, abs(arm.getTargetExt() - arm.getExt()) < 1.1)) {
            if (!queuer.isExecuted()) {
                claw.goTo(Claw.ClawStates.GIGA_OPEN);
                twist.twistTo(Twist.TwistStates.PARALLEL);
                flip.flipTo(Flip.FlipStates.RESET);
                arm.goTo(TelescopicArm.ArmStates.RETRACTED);
            }
        }
    }

    public void setArm(TelescopicArm.ArmStates targ, boolean p_async) {
        if (queuer.queue(p_async, abs(arm.getTargetExt() - arm.getExt()) < 1 && abs(arm.getTargetRot() - arm.getRot()) < 3) && !queuer.isExecuted() && !queuer.isFirstLoop())
            arm.goTo(targ);
    }

    public void setArm(double extension, double rot, boolean p_async) {
        if (queuer.queue(p_async, abs(arm.getTargetExt() - arm.getExt()) < 2 && abs(arm.getTargetRot() - arm.getRot()) < 3.5)/*&& abs(arm.getRotVel())<15*/ && !queuer.isExecuted() && !queuer.isFirstLoop())
            arm.goTo(extension, rot);
    }

    public void setArm(double extension, double rot, boolean p_async, Queuer queuer) {
        if (queuer.queue(p_async, abs(arm.getTargetExt() - arm.getExt()) < 4 && abs(arm.getTargetRot() - arm.getRot()) < 7)/*&& abs(arm.getRotVel())<15*/ && !queuer.isExecuted() && !queuer.isFirstLoop())
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
        if (queuer.queue(p_async, true) && !queuer.isExecuted() && !queuer.isFirstLoop())
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
        if (queuer.queue(p_async, abs(arm.getTargetExt() - arm.getExt()) < 2) && !queuer.isFirstLoop())
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

    public void followPath(Point end, Point ctrl1, Point ctrl2, boolean p_asynchronous, boolean reversed) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), ctrl1, ctrl2, end))
                        .setReversed(reversed)
                        .build();
                follower.followPath(path2, false);
            }
        }
    }

    public void followPath(Point end, Point ctrl1, boolean p_asynchronous, boolean reversed) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), ctrl1, end))
                        .setReversed(reversed)
                        .setTangentHeadingInterpolation()
                        .setReversed(reversed)
                        .build();
                follower.followPath(path2, false);
            }
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

    public void followPathNotTargeted(Point end, Point end2, double pathMaxVelMultipler, double headingInterp0, double headingInterp1, boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, targeted || !queuers.get(2).isEmpty())) {
            if ((follower.getPointFromPath(1) == null || new Point(follower.getPose()).distanceFrom(follower.getPointFromPath(1)) < 2) && !targeted && queuers.get(2).isEmpty()) {
                if (follower.getPose().getX() < 75 && follower.isVeloStable() && abs(BasicRobot.time - follower.lastStableTime()) > STABLIZE_TIME) {
                    Pose current = follower.getPose();
                    PathChain path2 = follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), end))
                            .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                            .setPathMaxVelMultiplier(pathMaxVelMultipler)
                            .build();
                    follower.followPath(path2);
                } else if (follower.isVeloStable() && abs(BasicRobot.time - follower.lastStableTime()) > STABLIZE_TIME) {
                    Pose current = follower.getPose();
                    PathChain path2 = follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), end2))
                            .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                            .setPathMaxVelMultiplier(pathMaxVelMultipler)
                            .build();
                    follower.followPath(path2);
                }
            }

        }
    }

    public void followPath(Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, double tValue) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .setPathEndTValueConstraint(tValue)
                        .build();
                follower.followPath(path2);
            }
        }
    }

    public void followPath(Point end, double headingInterp0, double headingInterp1, int headingInterp, boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2;
                if (headingInterp == 0) {
                    path2 = follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), end))
                            .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                            .build();
                } else {
                    path2 = follower.pathBuilder()
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

    public void followPath(Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, double zeroAccelMuilt, boolean hodlPoint) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .setZeroPowerAccelerationMultiplier(zeroAccelMuilt)
                        .build();
                follower.followPath(path2, hodlPoint);
            }
        }
    }

    public void followPath(Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, boolean hodlPoint, Queuer queuer) {
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
    public void followPath(Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, boolean hodlPoint, double zeroPowerMult, Queuer queuer) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .setZeroPowerAccelerationMultiplier(zeroPowerMult)
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
                follower.followPath(path2, holdEnd);
            }
        }
    }

    public void followPath(Point mid, Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, double decel, boolean holdEnd) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), mid, end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .setZeroPowerAccelerationMultiplier(decel)
                        .build();
                follower.followPath(path2, holdEnd);
            }
        }
    }

    public void followPath(Point mid, Point end, boolean p_asynchronous, double zeroMultiply, boolean holdEnd, double tValue, boolean isReverse) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), mid, end))
                        .setReversed(isReverse)
                        .setZeroPowerAccelerationMultiplier(zeroMultiply)
                        .setPathEndTValueConstraint(tValue)
                        .build();
                follower.followPath(path2, holdEnd);
            }
        }
    }

    public void followPath(Point mid, Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, double zeroMultiply, boolean holdEnd, double tValue) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), mid, end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .setZeroPowerAccelerationMultiplier(zeroMultiply)
                        .setPathEndTValueConstraint(tValue)
                        .build();
                follower.followPath(path2, holdEnd);
            }
        }
    }

    public void followPath(Point mid, Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, boolean holdEnd, Queuer queuer) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), mid, end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .build();
                follower.followPath(path2, holdEnd);
            }
        }
    }

    public void followPath(Point mid, Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, boolean holdEnd, double tValue) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), mid, end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .setPathEndTValueConstraint(tValue)
                        .build();
                follower.followPath(path2, holdEnd);
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
                follower.followPath(path2, false);
            }
        }
    }

    public void followPath(Point mid, Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, double tValue) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), mid, end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .setPathEndTValueConstraint(tValue)
                        .build();
                follower.followPath(path2, false);
            }
        }
    }
    public void followPath(Point mid, Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, double decel, Queuer queuer) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), mid, end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .setZeroPowerAccelerationMultiplier(decel)
                        .build();
                follower.followPath(path2, false);
            }
        }
    }
    public void followPath(Point mid, Point mid2, Point end, double headingInterp0, double headingInterp1, boolean p_asynchronous, double tValue, Queuer queuer) {
        if (queuer.queue(p_asynchronous, !follower.isBusy())) {
            if (!queuer.isExecuted()) {
                Pose current = follower.getPose();
                PathChain path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(current.getX(), current.getY(), Point.CARTESIAN), mid,mid2, end))
                        .setLinearHeadingInterpolation(headingInterp0, headingInterp1)
                        .setPathEndTValueConstraint(tValue)
                        .build();
                follower.followPath(path2, false);
            }
        }
    }

    public void lowerToGround(boolean p_async) {
        if (queuer.queue(p_async, TelescopicArm.ArmStates.INTAKE.getState()))
            arm.lowerToIntake();
    }
    public void setHangPowe(double power, boolean p_async, Queuer queuer) {
        if (queuer.queue(p_async, true))
            hang.setPower(power);
    }

    public void autoGrab(int color) {
        boolean isRB = false;
        if (queuer.queue(true, !isAutoGrab && (queuers.get(2).isEmpty() || Claw.ClawTargetStates.CLOSED.getState() || TelescopicArm.ArmStates.INTAKE.getState())||((color==1 || color==0)&&time-lastStartAUtoGrabTime>1.5))) {
            if (!targeted && queuers.get(2).isEmpty()) {
                if (!isAutoGrab) {
                    claw.goTo(Claw.ClawStates.OPEN);
                    if (color == 1)
                        cv.swapBlue();
                    else if (color == 0)
                        cv.swapRed();
                    else
                        cv.swapYellow();
                    cv.resetCenter();
                    flip.flipTo(Flip.FlipStates.AUTO_GRAH);
                    lastStartAUtoGrabTime = time;
                }
                if (isAutoGrab && ((follower.getVelocityMagnitude() < 3.6 && abs(arm.getVel()) < 1.7&& abs(follower.getVelocityPose().getHeading()) < 3.6) || (follower.isVeloStable() && abs(arm.getVel()) < 1.7))) {

                    double[] relCent = cv.getCenter().clone();

                    if (!targeted && !Arrays.equals(relCent, new double[]{0, 0, 0, 0})) {
                        isAutoGrab = true;
                        cv.resetCenter();
                        relCent[0] = (relCent[2] * Math.sin(arm.getRot() * PI / 180) + relCent[0] * Math.cos(arm.getRot() * PI / 180) - FOR_CONST) * FOR_MULT;
                        packet.put("relCent0", relCent[0]);
                        packet.put("relCent1", relCent[1]);
                        packet.put("rotVel", follower.getVelocityPose().getHeading());
                        if (follower.isTeleDrive())
                            follower.stopTeleopDrive();
                        else if (relCent[0] * relCent[0] + relCent[1] * relCent[1] < 200) {
                            if (time - lastMoveTime > MOVE_INTERVAL) {
//                            flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
//                            claw.goTo(Claw.ClawStates.OPEN);
                                targeted = true;
                                follower.stopTeleopDrive();
                                Vector2d relVect = new Vector2d(0, ((-relCent[1] + Math.signum(-relCent[1] - follower.getStableRotVelo().getY()*cv.getLatency()) * SIDE_CONST)) * SIDE_MULT
                                        - follower.getStableRotVelo().getY() * cv.getLatency()).rotated(follower.getPose().getHeading()/*-follower.getStableRotVelo().getHeading()*cv.getLatency()*/);
                                Vector2d relVect2 = new Vector2d(0, (-relCent[1] * SIDE_MULT))
                                        .rotated(follower.getPose().getHeading() - follower.getStableRotVelo().getHeading() * cv.getLatency());
                                Pose pos = follower.getPose();
                                pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                                Pose pos2 = follower.getPose();
                                pos2.add(new Pose(relVect2.getX(), relVect2.getY(), 0));
                                double newExt = Math.max(arm.getExt() + relCent[0] - (-arm.getVel() + follower.getStableRotVelo().getX()) * cv.getLatency(), MIN_EXT+2);

                                if (newExt > arm.getExt() + 8 || newExt > MAX_EXT || newExt == MIN_EXT+1) {
                                    targeted = false;
                                } else {
//                                    if (newExt > MIN_EXT + .1) {
                                    flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
//                                    }
                                    double head = follower.getPose().getHeading() /*- follower.getStableRotVelo().getHeading()*cv.getLatency()*/;
                                    if (follower.getCurrentPath() != null) {
                                        head = follower.getCurrentPath().getHeadingGoal(1);
                                    }
                                    Point curTarg = lastTarg;
                                    Point newTarg = new Point(pos2);
                                    if (curTarg == null) {
                                        follower.holdPoint(new BezierPoint(new Point(pos)), head);
                                    } else if (curTarg.distanceFrom(newTarg) > 0.05) {
                                        follower.holdPoint(new BezierPoint(new Point(pos)), head);
                                    }
//                                    if(newExt>arm.getTargetExt()+.25){
//                                        newExt+=.5;
//                                    }
                                    if (newExt < arm.getTargetExt() - .25) {
                                        newExt -= RETRACT_CONST;
                                    }
                                    arm.goToResetManual(newExt, Math.atan2(3, newExt + 12) * 180 / PI);
                                    twist.twistToAng(relCent[3]);
                                    packet.put("newExt", newExt);
                                    packet.put("relVect", relVect);
                                    packet.put("relAng", relCent[3]);
                                    packet.put("latency", cv.getLatency());
                                    lastTarg = new Point(pos2);
                                    lastMoveTime = time;
                                }
                            }
                        }
                    } else if (!targeted) {
//                    arm.manualGoTo(1, 0);
                    }
                } else {
                    if (arm.getTargetExt() < 1 || arm.getTargetExt() > 30 || arm.getTargetRot() < 1 || arm.getTargetRot() > 90) {
                        arm.goTo(TelescopicArm.ArmStates.AUTO_GRAB);
                        flip.flipTo(Flip.FlipStates.AUTO_GRAH);
                        claw.goTo(Claw.ClawStates.OPEN);
                        if (arm.getExt() < 5) {
                            twist.twistTo(Twist.TwistStates.PARALLEL);
                        }
                    }
                    packet.put("TARGWHENCHECK", arm.getTargetExt());
                    cv.resetCenter();
                    isAutoGrab = true;
                }
            }
        }
        if (targeted && time - twist.getLastTwisTime() > .1 && (!follower.isBusy() || lastTarg == null || lastTarg.distanceFrom(new Point(follower.getPose())) < 1) && ((abs(arm.getTargetExt() - arm.getExt()) < 1 && abs(arm.getVel()) > .25) || (abs(arm.getTargetExt() - arm.getExt()) < 0.5))) {
            if (!Flip.FlipStates.SUBMERSIBLE.getState()) {
                isRB = true;
                isAutoGrab = false;
                targeted = false;
            } else {
                isRB = true;
                isAutoGrab = false;
                targeted = false;
            }
        }
        if (isRB || !queuers.get(2).isEmpty()) {
            if (queuers.get(8).isEmpty() && TelescopicArm.ArmStates.HOVER.getState() &&(arm.getTargetRot()!=0 || !queuers.get(2).isEmpty()) && (!queuers.get(2).isEmpty() || TelescopicArm.ArmStates.HOVER.getState() || TelescopicArm.ArmStates.AUTO_GRAB.getState())) {
                if (!Flip.FlipStates.SUBMERSIBLE.getState() || time - twist.getLastTwisTime() < 0.2) {
                    queuers.get(2).addDelay(DROP_DELAY_TIME);
                }
                if (queuers.get(2).queue(false, true)) {
                    arm.lowerToIntake();
                }
                setFlip(Flip.FlipStates.SUBMERSIBLE, true, queuers.get(2));
                queuers.get(2).addDelay(DELAY_TIME);
                setClaw(Claw.ClawStates.CLOSED, false, queuers.get(2));
            }
            isAutoGrab = false;
        }
        for (var i : queuers) {
            if (!i.isEmpty()) {
                i.setFirstLoop(false);
            }
            if (!i.isEmpty() && i.isFullfilled()) {
                i.reset();
            }
        }
    }

    public void update() {
        super.update();
        if(!isTeleop)
            arm.update();
        claw.update();
        flip.update();
        if (!follower.isTeleDrive())
            follower.update();
        else
            follower.updatePose();
        twist.update();


    }
    public void twisPerp(){
        twist.twistTo(Twist.TwistStates.PERPENDICULAR);
    }

    public void teleOp(boolean isBlue) {
        if (queuer.isFirstLoop()) {
            if (isBlue)
                cv.swapBlue();
            else
                cv.swapRed();
            claw.goTo(Claw.ClawStates.OPEN);
            queuer.setFirstLoop(false);
        }
        boolean isY = gampad.readGamepad(op.gamepad1.y || op.gamepad2.y, "gamepad1_y", "high basket");
        boolean isB = gampad.readGamepad(op.gamepad1.b || op.gamepad2.b, "gamepad1_b", "specimen grab");
        boolean isA = gampad.readGamepad(op.gamepad1.a, "gamepad1_a", "retract slide(flat if from drop, vert if from grab)");
        boolean isA2 = gampad.readGamepad((op.gamepad2.a), "gamepad2_a", "retract slide(flat if from drop, vert if from grab)");

        boolean isX = gampad.readGamepad(op.gamepad1.x || (op.gamepad2.x&&!op.gamepad2.dpad_left), "gamepad1_x", "specimen drop");
        boolean isX2 = gampad.readGamepad(op.gamepad2.x, "gamepad1_x", "specimen drop");

        boolean isRB = gampad.readGamepad(op.gamepad1.right_bumper, "gamepad1_right_bumper", "down to grab /close claw/open claw");
        boolean isSuperRB = gampad.readGamepad(op.gamepad2.right_bumper, "gamepad2_right_bumper", "down to grab /close claw/open claw");
        boolean isLB = gampad.readGamepad(op.gamepad1.left_bumper || op.gamepad2.left_bumper, "gamepad1_left_bumper", "ground_hover");
        boolean isSuperY = gampad.readGamepad(op.gamepad2.y, "gamepad2_y", "down to grab /close claw/open claw");
        boolean isRD = gampad.readGamepad(op.gamepad1.dpad_right, "gamepad1_dpad_right", "auto grab red");
        boolean isLD = gampad.readGamepad(op.gamepad1.dpad_left, "gamepad1_dpad_left", "auto grab yellow");
        boolean isUD = gampad.readGamepad(op.gamepad1.dpad_up, "gamepad1_dpad_up", "auto grab blue");


        boolean isDD = op.gamepad1.dpad_down;
        boolean isDD2 = op.gamepad2.dpad_down;
        boolean isLD2 = op.gamepad2.dpad_left;

        if (Claw.ClawStates.GIGA_OPEN.getState() || TelescopicArm.ArmStates.SPECIMEN_GRAB.getState()) {
            driveConst = 0.3;
        } else if (TelescopicArm.ArmStates.HIGH_BUCKET.getState()) {
            driveConst = .7;
        } else if (!TelescopicArm.ArmTargetStates.SPECIMEN_GRAB.getState() && !TelescopicArm.ArmStates.SPECIMEN_GRAB.getState()) {
            driveConst = .7;
        }
        if(arm.getTargetExt() == HIGHBUCKET_EXTEND_POS){
            driveConst = .7;
        }

        if (follower.isTeleDrive() || (abs(op.gamepad1.left_stick_y) > 0.001 || abs(op.gamepad1.left_stick_x) > 0.001 || abs(op.gamepad1.right_stick_x) > 0.001)) {
            if (!follower.isTeleDrive()) {
                follower.startTeleopDrive();
                follower.breakFollowing();
            }
            follower.rawDriving(new Pose2d(-op.gamepad1.left_stick_y * driveConst * 1 / .7, op.gamepad1.left_stick_x * max(driveConst/.6, 0.4), -op.gamepad1.right_stick_x * driveConst*.7), arm.getExt());
//            isAutoGrab = false;
//            targeted = false;
        }
        double extend = op.gamepad1.right_trigger - op.gamepad1.left_trigger;
        if (abs(extend) > .1) {
            if (!isDD)
                arm.manualGoTo(extend, 0);
            else
                arm.manualGoTo(0, extend);
//            isAutoGrab = false;
        }
        if ((isDD && isY) || (isDD2 && isY)) {
            arm.goTo(TelescopicArm.ArmStates.LOW_BUCKET);
            flip.flipTo(Flip.FlipStates.BUCKET);
            twist.twistTo(Twist.TwistStates.PERPENDICULAR);
            isAutoGrab = false;
        } else if (isY || !queuers.get(1).isEmpty()) {
           /* if (TelescopicArm.ArmStates.HIGH_BUCKET.getState() && queuers.get(1).isEmpty()) {
                flip.flipTo(Flip.FlipStates.SPECIMEN);
                twist.twistTo(Twist.TwistStates.PERPENDICULAR);
            } else if (queuers.get(1).isEmpty()) {*/
            if(isY){
                for (var i : queuers)
                    i.reset();
            }
            setArm(0, HIGHBUCKET_PITCH_POS, false, queuers.get(1));
            queuers.get(1).addDelay(0.4);
            setArm(HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS, true, queuers.get(1));
            setFlip(Flip.FlipStates.BUCKET, true, queuers.get(1));
            setTwist(Twist.TwistStates.PERPENDICULAR, true, queuers.get(1));
//            }
            isAutoGrab = false;
            targeted = false;
        }
        if (isB && isDD) {
            arm.goTo(SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS);
            flip.flipTo(Flip.FlipStates.SPECIMEN_GRAB);
            twist.twistTo(Twist.TwistStates.PERPENDICULAR);
            isAutoGrab = false;
        } else if (isB && !Claw.ClawStates.CLOSED.getState()) {
            arm.goTo(SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS);
            flip.flipTo(Flip.FlipStates.SPECIMEN_GRAB);
            twist.twistTo(Twist.TwistStates.SPECIMEN);
            claw.goTo(Claw.ClawStates.GIGA_OPEN);
            isAutoGrab = false;
            driveConst = .3;
        } else if (isB) {
            arm.goTo(SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS);
            flip.flipTo(Flip.FlipStates.SPECIMEN_GRAB);
            twist.twistTo(Twist.TwistStates.PERPENDICULAR);
            isAutoGrab = false;
        }
        if (isA || !queuers.get(0).isEmpty() || !queuers.get(3).isEmpty() || !queuers.get(4).isEmpty()) {
            targeted = false;
            isAutoGrab = false;
            if (isA) {
                for (var i : queuers)
                    i.reset();
            }
            if (!queuers.get(0).isEmpty() || ((TelescopicArm.ArmStates.HIGH_BUCKET.getState() || TelescopicArm.ArmStates.LOW_BUCKET.getState()) && !Flip.FlipStates.RESET.getState())) {
                if (queuers.get(0).isEmpty() || isA) {
                    if ((TelescopicArm.ArmStates.INTAKE.getState() || TelescopicArm.ArmStates.HOVER.getState()) && arm.getRot() < 40) {
                        follower.stopTeleopDrive();
                        Vector2d relVect = new Vector2d(-5, 0).rotated(follower.getPose().getHeading());
                        Pose pos = follower.getPose();
                        pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                        follower.holdPoint(new BezierPoint(new Point(pos)), follower.getPose().getHeading());
                    }
                    if (TelescopicArm.ArmStates.HIGH_BUCKET.getState() || TelescopicArm.ArmStates.LOW_BUCKET.getState()) {
                        follower.stopTeleopDrive();
                        Vector2d relVect = new Vector2d(5, 0).rotated(follower.getPose().getHeading());
                        Pose pos = follower.getPose();
                        pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                        follower.holdPoint(new BezierPoint(new Point(pos)), follower.getPose().getHeading());
                    }
                }
                queuers.get(0).addDelay(0.3);
                setArm(TelescopicArm.ArmStates.RETRACTED, false, queuers.get(0));
                queuers.get(0).addDelay(0.3);
                setFlip(Flip.FlipStates.RESET, true, queuers.get(0));
                setTwist(Twist.TwistStates.PERPENDICULAR, true, queuers.get(0));
                queuers.get(0).addDelay(0.3);
                setArm(TelescopicArm.ArmStates.RETRACTED, false, queuers.get(0));
                isAutoGrab = false;
            } else if (!queuers.get(3).isEmpty() || ((TelescopicArm.ArmStates.HIGH_SPECIMEN.getState())&&arm.getTargetRot()==HIGHSPECIMEN_PITCH_POS)) {
                if (queuers.get(3).isEmpty()) {
                    follower.stopTeleopDrive();
                    Vector2d relVect = new Vector2d(-10, 0).rotated(follower.getPose().getHeading());
                    Pose pos = follower.getPose();
                    pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                    follower.holdPoint(new BezierPoint(new Point(pos)), follower.getPose().getHeading());
                    claw.goTo(Claw.ClawStates.OPEN);
                }
                queuers.get(3).addDelay(0.2);
                setFlip(Flip.FlipStates.RETRACT, false, queuers.get(3));
                setClaw(Claw.ClawStates.OPEN, true, queuers.get(3));
                queuers.get(3).addDelay(0.0);
                setArm(TelescopicArm.ArmStates.RETRACTED, false, queuers.get(3));
            } else if (!queuers.get(4).isEmpty() || TelescopicArm.ArmStates.INTAKE.getState()) {
                setArm(0, 15, false, queuers.get(4));
                queuers.get(4).addDelay(0.3);
                setFlip(Flip.FlipStates.RESET, true, queuers.get(4));
                queuers.get(4).addDelay(0.3);
                setTwist(Twist.TwistStates.PERPENDICULAR, true, queuers.get(4));
            } else {
                arm.goTo(TelescopicArm.ArmStates.RETRACTED);
                flip.flipTo(Flip.FlipStates.RESET);
                if (Claw.ClawStates.GIGA_OPEN.getState()&&!Claw.ClawTargetStates.CLOSED.getState())
                    claw.goTo(Claw.ClawStates.OPEN);
                isAutoGrab = false;
            }
        }
        if (isA2 && !isLD2) {
            arm.goTo(TelescopicArm.ArmStates.RETRACTED);
            flip.flipTo(Flip.FlipStates.RESET);

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
//                targeted = true;
                relCen[0] = relCen[2] * Math.sin(arm.getRot() * PI / 180) + relCen[0] * Math.cos(arm.getRot() * PI / 180) - 1.5;
                packet.put("relCent0", relCen[0]);
                packet.put("relCent1", relCen[1]);
            }
            lastReadTime = time;
        }
        if ((isLB && !op.gamepad1.dpad_down) || isRD || isUD || isAutoGrab) {
            if (isRD || isUD || isLB)
                claw.goTo(Claw.ClawStates.OPEN);
            if (isRD && isBlue)
                cv.swapBlue();
            else if (isRD)
                cv.swapRed();
            else if (isUD)
                cv.swapYellow();
            if (isRD || isLB || isUD) {
                cv.resetCenter();
                flip.flipTo(Flip.FlipStates.AUTO_GRAH);
                targeted = false;
                queuers.get(2).reset();
                queuers.get(0).reset();
            }
            if (isAutoGrab && TelescopicArm.ArmStates.HOVER.getState() && ((abs(follower.getVelocityPose().getHeading()) < 10 && abs(arm.getVel())<3) || (follower.isVeloStable() && abs(arm.getVel()) < 3))) {

                double[] relCent = cv.getCenter().clone();

                if (!targeted && !Arrays.equals(relCent, new double[]{0, 0, 0, 0})) {
                    isAutoGrab = true;
                    cv.resetCenter();
                    relCent[0] = (relCent[2] * Math.sin(arm.getRot() * PI / 180) + relCent[0] * Math.cos(arm.getRot() * PI / 180) - FOR_CONST) * FOR_MULT;
                    packet.put("relCent0", relCent[0]);
                    packet.put("relCent1", relCent[1]);
                    packet.put("rotVel", follower.getVelocityPose().getHeading());
                    if (follower.isTeleDrive())
                        follower.stopTeleopDrive();
                    else if (relCent[0] * relCent[0] + relCent[1] * relCent[1] < 200) {
                        if (time - lastMoveTime > MOVE_INTERVAL) {
//                            flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
//                            claw.goTo(Claw.ClawStates.OPEN);
                            targeted = true;
                            follower.stopTeleopDrive();
                            Vector2d relVect = new Vector2d(0, ((-relCent[1] + Math.signum(-relCent[1] - follower.getStableRotVelo().getY()) * SIDE_CONST)) * SIDE_MULT
                                    - follower.getStableRotVelo().getY() * cv.getLatency()).rotated(follower.getPose().getHeading()/*-follower.getStableRotVelo().getHeading()*cv.getLatency()*/);
                            Vector2d relVect2 = new Vector2d(0, (-relCent[1] * SIDE_MULT))
                                    .rotated(follower.getPose().getHeading() - follower.getStableRotVelo().getHeading() * cv.getLatency());
                            Pose pos = follower.getPose();
                            pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
                            Pose pos2 = follower.getPose();
                            pos2.add(new Pose(relVect2.getX(), relVect2.getY(), 0));
                            double newExt = Math.max(arm.getExt() + relCent[0] - (-arm.getVel() + follower.getStableRotVelo().getX()) * cv.getLatency(), MIN_EXT);

                            if (newExt > arm.getExt() + 8 || newExt > MAX_EXT) {
                                targeted = false;
                            } else {
//                                    if (newExt > MIN_EXT + .1) {
                                flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
//                                    }
                                double head = follower.getPose().getHeading() /*- follower.getStableRotVelo().getHeading()*cv.getLatency()*/;
                                if (follower.getCurrentPath() != null) {
                                    head = follower.getCurrentPath().getHeadingGoal(1);
                                }
                                Point curTarg = lastTarg;
                                Point newTarg = new Point(pos2);
                                if (curTarg == null) {
                                    follower.holdPoint(new BezierPoint(new Point(pos)), head);
                                } else if (curTarg.distanceFrom(newTarg) > 0.05) {
                                    follower.holdPoint(new BezierPoint(new Point(pos)), head);
                                }
//                                    if(newExt>arm.getTargetExt()+.25){
//                                        newExt+=.5;
//                                    }
                                if (newExt < arm.getTargetExt() - .25) {
                                    newExt -= RETRACT_CONST;
                                }
                                arm.goToResetManual(newExt, Math.atan2(3, newExt + 12) * 180 / PI);
                                twist.twistToAng(relCent[3]);
                                packet.put("newExt", newExt);
                                packet.put("relVect", relVect);
                                packet.put("relAng", relCent[3]);
                                packet.put("latency", cv.getLatency());
                                lastTarg = new Point(pos2);
                                lastMoveTime = time;
                            }
                        }
                    }
                } else if (!targeted) {
//                    arm.manualGoTo(1, 0);
                }
            } else {
                if (arm.getTargetExt() < 1 || arm.getTargetExt() > 30 || arm.getTargetRot() < 1 || arm.getTargetRot() > 90) {
                    arm.goTo(TelescopicArm.ArmStates.AUTO_GRAB);
                    flip.flipTo(Flip.FlipStates.AUTO_GRAH);
                    claw.goTo(Claw.ClawStates.OPEN);
                    if (arm.getExt() < 5) {
                        twist.twistTo(Twist.TwistStates.PARALLEL);
                    }
                }
                packet.put("TARGWHENCHECK", arm.getTargetExt());
                cv.resetCenter();
                isAutoGrab = true;
            }
        }
        if (targeted && time - twist.getLastTwisTime() > .1 && (!follower.isBusy() || lastTarg == null || lastTarg.distanceFrom(new Point(follower.getPose())) < 2) && ((abs(arm.getTargetExt() - arm.getExt()) < 2 && abs(arm.getVel()) > .25) || (abs(arm.getTargetExt() - arm.getExt()) < 0.5))) {
            if (!Flip.FlipStates.SUBMERSIBLE.getState()) {
                isRB = true;
                isAutoGrab = false;
                targeted = false;
            } else {
                isRB = true;
                isAutoGrab = false;
                targeted = false;
            }
        }

        if (isLD) {
            arm.goTo(TelescopicArm.ArmStates.AUTO_GRAB);
            flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
            twist.twistTo(Twist.TwistStates.PARALLEL);
            isAutoGrab = false;
        }
        if (op.gamepad1.dpad_down && isRB) {
            twist.iterateTwist(1);
        }
        if (isRB || !queuers.get(2).isEmpty() || !queuers.get(8).isEmpty()) {
            if (queuers.get(8).isEmpty() && TelescopicArm.ArmStates.HOVER.getState() &&(arm.getTargetRot()!=0 || !queuers.get(2).isEmpty()) && (!queuers.get(2).isEmpty() || TelescopicArm.ArmStates.HOVER.getState() || TelescopicArm.ArmStates.AUTO_GRAB.getState())) {
                if (!Flip.FlipStates.SUBMERSIBLE.getState() || time - twist.getLastTwisTime() < 0.2) {
                    queuers.get(2).addDelay(DROP_DELAY_TIME);
                }
                if (queuers.get(2).queue(false, true)) {
                    arm.lowerToIntake();
                }
                setFlip(Flip.FlipStates.SUBMERSIBLE, true, queuers.get(2));
                queuers.get(2).addDelay(DELAY_TIME);
                setClaw(Claw.ClawStates.CLOSED, false, queuers.get(2));
            } else if (((arm.getTargetExt() == HIGHBUCKET_EXTEND_POS|| arm.getTargetExt() == LOWBUCKET_EXTEND_POS || arm.getExt()>HIGHBUCKET_EXTEND_POS-4) && Claw.ClawStates.CLOSED.getState()) || !queuers.get(8).isEmpty()) {
                if (queuers.get(8).queue(false, true)) {

                }
                queuers.get(8).addDelay(0.7);
                setFlip(Flip.FlipStates.AUTO_GRAH, false, queuers.get(8));
                queuers.get(8).addDelay(0.14);
                setTwist(Twist.TwistStates.SPECIMEN, true, queuers.get(8));
                queuers.get(8).addDelay(DROP_DEL);
                setClaw(Claw.ClawStates.OPEN, true, queuers.get(8));
                queuers.get(8).addDelay(0.1);
                setFlip(Flip.FlipStates.SPECIMEN_GRAB, true, queuers.get(8));


            } else if (isRB && Claw.ClawStates.CLOSED.getState() &&
                    (arm.getRot()>150)) {
                claw.goTo(Claw.ClawStates.GIGA_OPEN);
                twist.twistTo(Twist.TwistStates.SPECIMEN);
            } else if (isRB && Claw.ClawStates.CLOSED.getState()) {
                claw.goTo(Claw.ClawStates.OPEN);
            } else {
                claw.goTo(Claw.ClawStates.CLOSED);
            }
            isAutoGrab = false;
        }
        if (isDD2 && isSuperRB || !queuers.get(6).isEmpty()) {
            if (queuers.get(6).isEmpty()) {
                if (follower.isTeleDrive()) {
                    grabPoint = follower.getPose();
                    follower.stopTeleopDrive();
                } else {
                    grabPoint = new Pose(grabPoint.getX() + CYCLE_OFFSET_X, grabPoint.getY() + CYCLE_OFFSET_Y, grabPoint.getHeading() + H_OFFSET);
                }
            }
            Vector2d dist = new Vector2d(DIST1_X, DIST1_Y);
            Vector2d dist2 = new Vector2d(DIST2_X, DIST2_Y);
            Vector2d dist12 = new Vector2d(DIST12_X, DIST12_Y);
            Vector2d dist3 = new Vector2d(DIST3_X, DIST3_Y);
            Vector2d dist4 = new Vector2d(DIST4_X, DIST4_Y);
            Vector2d dist5 = new Vector2d(DIST5_X, DIST5_Y).rotated(grabPoint.getHeading());

            dist = dist.rotated(grabPoint.getHeading());
            dist2 = dist2.rotated(grabPoint.getHeading());
            dist12 = dist12.rotated(grabPoint.getHeading());
            dist3 = dist3.rotated(grabPoint.getHeading());
            dist4 = dist4.rotated(grabPoint.getHeading());
//            Vector2d two = new Vector2d(DISTTWO_X,DISTTWO_Y).rotated(grabPoint.getHeading());
//            followPath(new Point(dist3.getX()+grabPoint.getX()+two.getX(), dist3.getY()+grabPoint.getY()+two.getY(),1),
//                    grabPoint.getHeading(), grabPoint.getHeading(),false, false, queuers.get(6));
            setClaw(Claw.ClawStates.CLOSED, false, queuers.get(6));
            followPath(
                    new Point(dist2.getX()+grabPoint.getX(), dist2.getY()+grabPoint.getY(),1),
                    new Point(dist12.getX()+grabPoint.getX(), dist12.getY()+grabPoint.getY(),1),
                    new Point(dist.getX() + grabPoint.getX(), dist.getY() + grabPoint.getY(), Point.CARTESIAN),
                    grabPoint.getHeading(), grabPoint.getHeading(), false, 0.9, queuers.get(6));
            setArm(0, TelescopicArm.ArmStates.HIGH_SPECIMEN.getPitchPos(), true, queuers.get(6));
            queuers.get(6).addDelay(RAISE_DELAY);
            setArm(TelescopicArm.ArmStates.HIGH_SPECIMEN.getExtendPos(), TelescopicArm.ArmStates.HIGH_SPECIMEN.getPitchPos(), true, queuers.get(6));
            setTwist(Twist.TwistStates.PARALLEL, true, queuers.get(6));
            setFlip(Flip.FlipStates.SPECIMEN, true, queuers.get(6));
            setClaw(Claw.ClawStates.GIGA_OPEN, false, queuers.get(6));
            queuers.get(6).addDelay(0.4);
            setArm(TelescopicArm.ArmStates.SPECIMEN_GRAB, true, queuers.get(6));
            followPath(
                    new Point(dist3.getX()+grabPoint.getX()+dist4.getX(), dist3.getY()+dist4.getY()+grabPoint.getY(),1),
                    new Point(dist3.getX()+grabPoint.getX()+dist5.getX(), dist3.getY()+dist5.getY()+grabPoint.getY(),1),
                    new Point(dist3.getX()+grabPoint.getX(), dist3.getY()+grabPoint.getY(),1),
                    grabPoint.getHeading(), grabPoint.getHeading(), false,0.9, queuers.get(6));
            setTwist(Twist.TwistStates.SPECIMEN, true, queuers.get(6));
            setFlip(Flip.FlipStates.SPECIMEN_GRAB, true, queuers.get(6));
//            setClaw(Claw.ClawStates.CLOSED, false, queuers.get(6));
        } else if (isSuperRB) {
            if (Claw.ClawStates.CLOSED.getState())
                claw.goTo(Claw.ClawStates.OPEN);
            else
                claw.goTo(Claw.ClawStates.CLOSED);
        }

        if (isDD2 && isSuperY) {
            arm.goTo(TelescopicArm.ArmStates.HIGH_BUCKET);
            flip.flipTo(Flip.FlipStates.BUCKET);
            twist.twistTo(Twist.TwistStates.PERPENDICULAR);
//            isAutoGrab = false;
        }
        if (isLD2 && isX2 || !queuers.get(9).isEmpty()) {
            setFlip(Flip.FlipStates.SPECIMEN, true, queuers.get(9));
            setTwist(Twist.TwistStates.SPECIMEN, true, queuers.get(9));
            setArm(HANGEXT1, HANGROT1, false, queuers.get(9));
            setHangPowe(-1,true, queuers.get(9));
            queuers.get(9).addDelay(0.9);
            setHangPowe(0,true, queuers.get(9));
            queuers.get(7).reset();
        }
        if (isLD2 && isA2 || !queuers.get(7).isEmpty()) {
            queuers.get(7).queue(false, true);
            queuers.get(7).addDelay(1);
            setArm(HANGEXT3, HANGROT3, false, queuers.get(7));
            setFlip(Flip.FlipStates.SUBMERSIBLE, true, queuers.get(7));
            setArm(HANGEXT2, HANGROT2, true, queuers.get(7));
        }
        double hangPow = op.gamepad2.right_trigger - op.gamepad2.left_trigger;
        if(abs(hangPow)>.05){
            queuers.get(9).reset();
            arm.setPowers(1, 0);
            hang.setPower(hangPow);
        }
        else{
            if(queuers.get(9).isEmpty()) {
                hang.setPower(0);
            }

            arm.update();
        }
        for (var i : queuers) {
            if (!i.isEmpty()) {
                i.setFirstLoop(false);
            }
            if (!i.isEmpty() && i.isFullfilled()) {
                i.reset();
            }
        }
        update();
    }
}
