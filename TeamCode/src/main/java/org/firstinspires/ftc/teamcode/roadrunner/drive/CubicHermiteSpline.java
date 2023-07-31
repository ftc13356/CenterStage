package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Double.max;
import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
@Config
public class CubicHermiteSpline {
    private double t;
    private ArrayList<Vector2d> coeffs;
    private double length = 0;
    private double duration = 0;
    private double lengthResolution = 400;
    private double numericDerivResolution = 20, travelDist = 0;
    private double AVG_SCALE_FACTOR = 1.2;
    double numericIntegral, numericT = 0;
    private double numericIntegralResolution = 0.001;
    private RFTrajectory traj;
    double targetDistance = 0;
    Vector2d endPos, endVel, startPos, startVel, lastPos, lastIntegralPos;
    Pose2d targetPose = currentPose;
    Pose2d targetVelocity = currentVelocity;
    //    Pose2d targetAcceleration;
    Pose2d instantaneousVelocity;
    Pose2d instantaneousAcceleration;
    public static double LOOPINESS = 1.4;

    public CubicHermiteSpline(Vector2d p_startPos, Vector2d p_startVel, Vector2d p_endVel, Vector2d p_endPos, RFTrajectory p_traj) {
        traj = p_traj;
        startPos = p_startPos;
        startVel = p_startVel;
        endPos = p_endPos;
        endVel = p_endVel;
        length = p_startPos.distTo(p_endPos);
        //calc approxDuration
        packet.put("length0", length);
        double loopyStorage = LOOPINESS;
        duration = traj.calculateSegmentDuration(length * AVG_SCALE_FACTOR);
        if(p_endVel.norm()*duration*LOOPINESS>MAX_VEL){
            LOOPINESS = MAX_VEL/(p_endVel.norm()*duration*LOOPINESS);
        }
        Vector2d p_startVelo = p_startVel.times(LOOPINESS* duration);
        Vector2d p_endVelo = p_endVel.times(LOOPINESS* duration);
        boolean shortSegment = p_endVelo.norm()-p_startVelo.norm()>length/AVG_SCALE_FACTOR;
        if(shortSegment){
            p_endVelo.div(LOOPINESS);
        }

        packet.put("duration", duration);
        coeffs = new ArrayList<>();
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        Vector2d lastPose = p_startPos;
        length = 0;
        for (int i = 0; i < lengthResolution; i++) {
            Vector2d newPos = poseAt((i + 1) / lengthResolution, coeffs);
            length += newPos.distTo(lastPose);
            lastPose = newPos;
        }
        packet.put("length1", length);
        //calc duration
        duration = traj.calculateSegmentDuration(length);
        double newEndVel = traj.getEndVelMag();
        p_startVelo = p_startVel.times(LOOPINESS* duration);
        p_endVelo = p_endVel.times(LOOPINESS* duration);
        p_endVelo = p_endVelo.times(newEndVel/p_endVelo.norm());
        coeffs.clear();
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        lastPose = p_startPos;
        length = 0;
        for (int i = 0; i < lengthResolution; i++) {
            Vector2d newPos = poseAt((i + 1) / lengthResolution, coeffs);
            length += newPos.distTo(lastPose);
            lastPose = newPos;
        }
        packet.put("coeffs0", coeffs.get(0));
        packet.put("coeffs1", coeffs.get(1));
        packet.put("coeffs2", coeffs.get(2));
        packet.put("coeffs3", coeffs.get(3));
        //calc duration
        duration = traj.calculateSegmentDuration(length);
        p_startVelo = p_startVel.times(LOOPINESS* duration);
        p_endVelo = p_endVel.times(LOOPINESS* duration);
        p_endVelo = p_endVelo.times(newEndVel/p_endVelo.norm());
        coeffs.clear();
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        lastPos = currentPose.vec();
        lastIntegralPos = lastPos;
        numericDerivResolution*=duration;
        LOOPINESS = loopyStorage;
    }

    public double getLength() {
        return length;
    }

    public double getRemDistance() {
        travelDist += currentPose.vec().distTo(lastPos);
        lastPos = currentPose.vec();
        return length - travelDist;
    }

    public Vector2d poseAt(double p_t, ArrayList<Vector2d> coeffs) {
        return coeffs.get(0).plus(coeffs.get(1).times(p_t)).plus(coeffs.get(2).times(p_t * p_t)).plus(coeffs.get(3).times(p_t * p_t * p_t));
    }

    public Vector2d derivAt(double p_t, ArrayList<Vector2d> coeffs) {
        return coeffs.get(1).plus(coeffs.get(2).times(2 * p_t)).plus(coeffs.get(3).times(3 * p_t * p_t));
    }

    public Vector2d scndDerivAt(double p_t, ArrayList<Vector2d> coeffs) {
        return coeffs.get(2).times(2).plus(coeffs.get(3).times(6 * p_t));
    }

    public double calcAngularVel(Vector2d vel, Vector2d accel) {
        if (vel.getY() == 0) {
            vel = new Vector2d(vel.getX(), 0.00001);
        }
        if (vel.getX() == 0) {
            vel = new Vector2d(0.00001, vel.getY());
        }

        return (accel.getY() / vel.getX() - (accel.getX()) * (vel.getY()))
                / ((vel.getY() * vel.getY()) / (vel.getX() * vel.getX()) + 1);
    }

    public double power(double v1, int v2) {
        double product = v1;
        for (int i = 1; i < v2; i++) {
            product *= v1;
        }
        return product;
    }

    public double approximateT(double distance) {
        int switchTimes = 1;
        boolean lastDirection = true;
        numericT = 0;
        numericIntegral = 0;
        lastIntegralPos = startPos;
        double error = distance - numericIntegral;
        if (error < 0) {
            lastDirection = false;
        }
        while (abs(error) > numericIntegralResolution) {
            if (error > 0) {
                if (!lastDirection) {
                    lastDirection = true;
                    switchTimes++;
                }
                numericT += Math.max(power(0.1, switchTimes), numericIntegralResolution);
                Vector2d newPos = poseAt(numericT, coeffs);
                numericIntegral += lastIntegralPos.distTo(newPos);
                lastIntegralPos = newPos;
            } else {
                if (lastDirection) {
                    lastDirection = false;
                    switchTimes++;
                }
                numericT -= Math.max(power(0.1, switchTimes), numericIntegralResolution);
                Vector2d newPos = poseAt(numericT, coeffs);
                numericIntegral -= lastIntegralPos.distTo(newPos);
                lastIntegralPos = newPos;
            }
            if (switchTimes > 5) {
                break;
            }
            error = distance - numericIntegral;
        }
        return numericT;
    }

    public ArrayList<Vector2d> getCurDerivs() {
        double p_t = approximateT(targetDistance);
        Vector2d pose = poseAt(p_t, coeffs);
        Vector2d deriv = derivAt(p_t, coeffs);
        Vector2d scnDeriv = scndDerivAt(p_t, coeffs);
        ArrayList<Vector2d> derivs = new ArrayList<>();
        derivs.add(pose);
        derivs.add(deriv);
        derivs.add(scnDeriv);
        return derivs;
    }

    public void calculateTargetPoseAt(double distance) {
        double p_t = max(min(approximateT(distance), 1),    0);
        packet.put("DIst", distance);
        packet.put("approxT", p_t);
        targetDistance = distance;
        Vector2d pose = poseAt(p_t, coeffs);
        Vector2d deriv = derivAt(p_t, coeffs);
        Vector2d scnDeriv = scndDerivAt(p_t, coeffs);
        double angle = Math.atan2(deriv.getY(), deriv.getX());
        targetPose = new Pose2d(pose, angle);
        Vector2d velo;
        double ttoTimeRatio = traj.timeToTRatio(deriv.norm());
        packet.put("timeToTRatio", ttoTimeRatio);
        packet.put("derivX", deriv.getX());
        //calculated target Acceleration, not needed for PID
        double newT =  p_t+1/numericDerivResolution;
        double numericDerivResolution = this.numericDerivResolution;
        if(newT>1){
            newT = 1;
            numericDerivResolution = 1/(1-p_t);
        }
        Vector2d deriv2 = derivAt(newT, coeffs);
        double angle2 = Math.atan2(deriv2.getY(), deriv2.getX());
        double angularVel2 = (angle2-angle)*numericDerivResolution * ttoTimeRatio;
        if(abs(angularVel2) > MAX_ANG_VEL){
            angularVel2 *= MAX_ANG_VEL/abs(angularVel2);
        }
//        deriv = deriv.times(ttoTimeRatio);
        targetVelocity = new Pose2d(deriv.times(ttoTimeRatio), angularVel2 );
        double magSquared = targetVelocity.vec().norm()*targetVelocity.vec().norm();
        double angleMagSquared = targetVelocity.getHeading()*targetVelocity.getHeading()*TRACK_WIDTH*TRACK_WIDTH*0.25;
        double ratio = targetVelocity.vec().norm()/sqrt(magSquared+angleMagSquared);
        if(ratio<1) {
            targetVelocity.times(ratio);
        }
//        packet.put("ratio",ratio);
    }

    public void calculateInstantaneousTargetPose() {
        Vector2d curPos = currentPose.vec();
        Vector2d curVel = currentVelocity.vec();
        double remDuration = traj.remainingSegmentTime(getRemDistance());

        if (remDuration < 1) {
            remDuration = 1;
        }
        Vector2d curVelo = curVel.div(remDuration);
        Vector2d endVelo = endVel.div(remDuration);
        ArrayList<Vector2d> tempCoeffs = new ArrayList();
        tempCoeffs.add(curPos);
        tempCoeffs.add(curVelo);
        tempCoeffs.add(curPos.times(-3).minus(curVelo.times(2)).minus(endVelo).plus(endPos.times(3)));
        tempCoeffs.add(curPos.times(2).plus(curVelo).plus(endVelo).minus(endPos.times(2)));
        Vector2d accel = tempCoeffs.get(2).times(remDuration * remDuration * 0.5);
        double newT =  1/numericDerivResolution;
        double numericDerivResolution = this.numericDerivResolution;
        if(newT>0.5){
            newT = 0.5;
            numericDerivResolution = 0.5;
        }
        Vector2d curVel2 = derivAt(newT, tempCoeffs).times(remDuration);
        Vector2d curVel3 = derivAt(2*newT, tempCoeffs).times(remDuration);
        double angle2 = Math.atan2(curVel2.getY(), curVel2.getX());
        double angle = Math.atan2(curVel.getY(), curVel.getX());
        double angle3 = Math.atan2(curVel3.getY(), curVel3.getX());
        double dT = newT * remDuration;
        double angularVel = (angle2 - angle) / dT;
        double angularVel2 = (angle3 - angle2) / dT;
        instantaneousVelocity = new Pose2d(curVel, angularVel);
        angularVel = (angularVel2 - angularVel) / dT;
        instantaneousAcceleration = new Pose2d(accel, angularVel);
    }

}
