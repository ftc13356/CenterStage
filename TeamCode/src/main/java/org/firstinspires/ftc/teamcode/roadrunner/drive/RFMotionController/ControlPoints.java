package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.signum;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class ControlPoints {
    ArrayList<ControlPoint> points;
    double controlPointRes = 100;
    double totalTime = 0;
    double startTime =0;

    public ControlPoints(QuinticHermiteSpline p_spline, Pose2d p_v0, Pose2d p_v1) {
        startTime = time;
        points = new ArrayList<>();
        points.add(new ControlPoint(p_spline.valsAt(0), p_v0.vec().norm(), 0, p_v0.getHeading()));
        points.get(0).setPrevK(points.get(0).getK());
        Object[] times = p_spline.binRecursion();
        for (int i = 1; i < times.length-1; i++) {
            points.add(points.get(points.size() - 1).forwardTraverse(p_spline.valsAt((double)times[i])));
            points.get(points.size() - 1).setPrevK(points.get(points.size() - 2).getK());
            points.get(points.size() - 2).setNextK(points.get(points.size() - 1).getK());
        }
        points.add(new ControlPoint(p_spline.valsAt(1), p_v1.vec().norm(), 0, p_v1.getHeading()));
        points.get(points.size() - 1).setPrevK(points.get(points.size() - 2).getK());
        points.get(points.size() - 2).setNextK(points.get(points.size() - 1).getK());
        points.get(points.size() - 1).setNextK(points.get(points.size() - 1).getK());
        double lastSignum = signum(points.get(points.size() - 1).getTime());
        for (int i = 2; i < times.length; i++) {
            int ind = times.length - i;
            points.set(ind, points.get(ind + 1).backwardTraverse(points.get(ind)));
            double signum = signum(points.get(ind).getTime());
            if (signum != lastSignum && totalTime == 0) {
                totalTime = abs(points.get(ind).getTime()) + abs(points.get(ind).getOtherTime());
            }
            lastSignum = signum;
        }
    }

    /**
     * targetPose, targetVel, targetAccel, targetHead(vel, accel)
     * @return targetPose, targetVel, targetAccel, targetHead(vel, accel)
     */
    public Pose2d[] getTargets() {
        double[] ind = binPose(points, currentPose.vec());
        ControlPoint p1 = points.get((int) ind[0]), p2 = points.get((int) ind[0] + 1);
        Pose2d targetPose = p1.getPose().times(ind[1]).plus(p2.getPose().times(1-ind[1]));
        Pose2d targetVel = p1.getVelo().times(ind[1]).plus(p2.getVelo().times(1-ind[1]));
        Pose2d targetAccel = p1.getAccel().times(ind[1]).plus(p2.getAccel().times(1-ind[1]));
        return new Pose2d[]{targetPose, targetVel, targetAccel};
    }

    public Pose2d getTargetPoint(){
        return points.get(points.size()-1).getPose();
    }
    public Vector2d getTargetVel(){
        return points.get(points.size()-1).getVeloVec();
    }
    /**
     * binary search for first index of the range [a,a+1] that contains target time as well as the weights of how close it is to each border from 0-1
     *
     * @param p_arr array to search through
     * @param p_t   time to search for
     * @return array of {a, weight1}
     */
    public double[] bin(ArrayList<ControlPoint> p_arr, double p_t) {
        int upBound = p_arr.size() - 1, downBound = 0;
        while (upBound > downBound + 1) {
            int m = (upBound + downBound) / 2;
            double t = p_arr.get(m).getTime();
            if(t<0){
                t = startTime+totalTime-t;
            }else{
                t = startTime+t;
            }
            if (t> p_t) {
                upBound = m;
            } else if (t< p_t) {
                downBound = m;
            } else {
                downBound = m;
                break;
            }
        }
        double tDown = p_arr.get(downBound).getTime();
        double tUp = p_arr.get(upBound).getTime();


        double downDist = p_t - tDown;
        double upDist = tUp - p_t;
        double[] weight = {downDist / (downDist + upDist), upDist / (downDist + upDist)};
        return new double[]{downBound, weight[0]};
    }

    /**
     * binary search for between which two points the target position is
     * @param p_arr controlPoints to search
     * @param p_pose target position
     * @return double array with {firstControlPoint, how close to first control point}
     */
    public double[] binPose(ArrayList<ControlPoint> p_arr, Vector2d p_pose) {
        int upBound = p_arr.size() - 1, downBound = 0;
        while (upBound > downBound + 1) {
            int m = (upBound + downBound) / 2;
            if (p_arr.get(m).getPoseVec().distTo(p_pose) > p_arr.get(m+1).getPoseVec().distTo(p_pose)) {
                downBound = m;
            } else if (p_arr.get(m).getPoseVec().distTo(p_pose) < p_arr.get(m+1).getPoseVec().distTo(p_pose)) {
                upBound = m;
            } else {
                downBound = m;
                break;
            }
        }
        double downDist = p_pose.distTo(p_arr.get(downBound).getPoseVec());
        double upDist = p_pose.distTo(p_arr.get(upBound).getPoseVec());
        double weight = downDist / (downDist + upDist);
        return new double[]{downBound, weight};
    }
    //generate short cubic hermite spline to estimate, 5x error min of 2 control points ahead
    public Pose2d[] getInstantaneousTarget(){
        double[] ind = binPose(points, currentPose.vec());
        ControlPoint p1 = points.get((int) ind[0]), p2 = points.get((int) ind[0] + 1);
        Pose2d targetPose = p1.getPose().times(ind[1]).plus(p2.getPose().times(1-ind[1]));
        Pose2d targetVel = p1.getVelo().times(ind[1]).plus(p2.getVelo().times(1-ind[1]));
        Pose2d targetAccel = p1.getAccel().times(ind[1]).plus(p2.getAccel().times(1-ind[1]));
        return new Pose2d[]{targetPose, targetVel, targetAccel};
    }

}
