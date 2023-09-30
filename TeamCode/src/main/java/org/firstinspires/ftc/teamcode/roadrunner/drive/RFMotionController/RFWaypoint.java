package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class RFWaypoint {
    private Pose2d target;
    private double endTangent;
    private double endVelocity;
    private int definedness;
    private double startTangentMag, endTangentMag, startAccel, endAccel, endCurvature;


    public RFWaypoint(Pose2d p_target, double p_endTangent, double p_endVelocity, int definedness) {
        createRFWaypoint(p_target, p_endTangent, p_endVelocity, definedness);
    }

    public RFWaypoint(Pose2d p_target, double p_endTangent, double p_endVelocity) {
        createRFWaypoint(p_target, p_endTangent, p_endVelocity, 3);
    }

    public RFWaypoint(Pose2d p_target, double p_endTangent) {
        createRFWaypoint(p_target, p_endTangent, 1000, 2);
    }

    public RFWaypoint(Pose2d p_target) {
        createRFWaypoint(p_target, 1000, 1000, 1);
    }

    public RFWaypoint(Vector2d p_target) {
        createRFWaypoint(new Pose2d(p_target, 1000), 1000, 1000, 0);
    }

    public void createRFWaypoint(Pose2d p_target, double p_endTangent, double p_endVelocity, int p_definedness) {
        target = p_target;
        endTangent = p_endTangent;
        endVelocity = p_endVelocity;
        definedness = p_definedness;
    }

    public void changeTo(RFWaypoint p_newWaypoint) {
        target = p_newWaypoint.target;
        endTangent = p_newWaypoint.endTangent;
        endVelocity = p_newWaypoint.endVelocity;
        definedness = p_newWaypoint.definedness;
    }

    public Pose2d getTarget() {
        return target;
    }

    public void setTarget(Pose2d p_target) {
        target = p_target;
    }

    public double getEndTangent() {
        return endTangent;
    }

    public void setEndTangent(double p_endTangent) {
        endTangent = p_endTangent;
    }

    public Vector2d getEndVelocityVec(){
        packet.put("endTangentCalc", endTangent);
        packet.put("endVeloCalc", endVelocity);
        return new Vector2d(cos(endTangent)*endVelocity, sin(endTangent)*endVelocity);
    }

    public double getEndVelocity() {
        return endVelocity;
    }

    public void setEndVelocity(double p_endVelocity) {
        endVelocity = p_endVelocity;
    }

    public int getDefinedness() {
        return definedness;
    }

    public Vector2d[] getSplineCoeffs(){
        Vector2d[] coeffs = {new Vector2d(),new Vector2d(),new Vector2d(),new Vector2d(),new Vector2d(),new Vector2d()};
        coeffs[0]=currentPose.vec();
        coeffs[1] = currentPose.vec().times(startTangentMag*currentVelocity.vec().norm());
        coeffs[2] = solveForCurvatureMaintaingAccel(currentVelocity.getHeading(), currentVelocity.getX(), currentVelocity.getY(), startAccel);
        coeffs[3] = getTarget().vec();
        coeffs[4] = getEndVelocityVec();
        packet.put("endCurv", endCurvature);
        packet.put("endVelo", coeffs[4]);
        packet.put("endAccel", endAccel);

        coeffs[5] = solveForCurvatureMaintaingAccel(endCurvature, coeffs[4].getX(), coeffs[4].getY(), endAccel);
        for(int i=0;i<coeffs.length; i++){
            packet.put("inpCoeffs"+i, coeffs[i]);
        }
        return coeffs;
    }
    public Vector2d[] getSplineCoeffs(Pose2d currentPose, Pose2d currentVelocity){
        Vector2d[] coeffs = {new Vector2d(),new Vector2d(),new Vector2d(),new Vector2d(),new Vector2d(),new Vector2d()};
        coeffs[0]=currentPose.vec();
        coeffs[1] = currentPose.vec().times(startTangentMag/currentPose.vec().norm());
        coeffs[2] = solveForCurvatureMaintaingAccel(currentVelocity.getHeading(), currentVelocity.getX(), currentVelocity.getY(), startAccel);
        coeffs[3] = getTarget().vec();
        coeffs[4] = getEndVelocityVec();
        coeffs[5] = solveForCurvatureMaintaingAccel(endCurvature, coeffs[4].getX(), coeffs[4].getY(), endAccel);
        return coeffs;
    }

    /**
     * TODO: rewrite this 
     * @param k
     * @param dx
     * @param dy
     * @param a
     * @return
     */
    public Vector2d solveForCurvatureMaintaingAccel(double k, double dx, double dy, double a){
        double n = k*pow(dx*dx+dy*dy, 1.5);
        double b = (2*n*dy+dx*dx)/(dx*dx);
        double A = dy/(dx*dx);
        double c = (n*n-a*dx*dx)/(dx*dx);
        double ddx = (-b+sqrt(b*b-4*A*c))/(2*A);
        double ddy = sqrt(a-ddx*ddx);
        if(dx==0&&dy==0){
            return new Vector2d(a*cos(k), a*sin(k));
        }
        if(A==0){
            ddx = -b+sqrt(b*b-4*A*c) * 10000;
            ddy = sqrt(a-ddx*ddx);
        }
        return new Vector2d(a*(ddx*ddx)/(ddx*ddx+ddy*ddy),a*(ddy*ddy)/(ddx*ddx+ddy*ddy));
    }
}
